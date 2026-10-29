"""Supplied acquisition, feedback, plots and evaluation; exercises live in the notebook."""
from collections import Counter
from pathlib import Path
import csv

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from calibration_pipeline.eflesh_calibration import knn as production

LABELS = list(production.CLASSES)
CHANNELS = [f's{s}{a}' for s in range(4) for a in 'xyz']


def _run_checks(name, checks):
    try:
        checks()
    except NotImplementedError:
        print(f'{name}: PENDING — fill the blank body, then rerun this cell.')
        return False
    except Exception as exc:
        print(f'{name}: CHECK FAILED — {type(exc).__name__}: {exc}')
        return False
    print(f'{name}: PASS — results match the reference.')
    return True


def _rejects(function, *args):
    try:
        function(*args)
    except ValueError:
        return
    raise AssertionError('Expected ValueError for invalid input.')


def verify_average(function):
    def checks():
        rng = np.random.default_rng(23)
        for n in (5, 6, 19, 43):
            x = rng.normal(size=(n, 12))
            result = function(x)
            assert np.shape(result) == (n-4, 12), 'Wrong output shape.'
            np.testing.assert_allclose(result, production.moving_average(x), atol=1e-12)
        x = np.zeros((10, 12)); x[4, 0] = 5
        original = x.copy()
        np.testing.assert_allclose(function(x), production.moving_average(x))
        np.testing.assert_array_equal(x, original, err_msg='Do not mutate input.')
        np.testing.assert_allclose(function(x)[:2], function(x[:6]))
    return _run_checks('moving_average', checks)


def verify_feature(function, average, average_ready):
    if not average_ready:
        print('feature: PENDING — finish moving_average first.'); return False
    def checks():
        # Confirm the student's feature calls their notebook moving_average.
        namespace = function.__globals__
        saved = namespace.get('moving_average')
        calls = []
        def tracked(x):
            calls.append(True)
            return average(x)
        namespace['moving_average'] = tracked
        try:
            rng = np.random.default_rng(31)
            for _ in range(4):
                x, base = rng.normal(size=(19,12)), rng.normal(size=12)
                original = x.copy(); original_base = base.copy()
                result = function(x, base)
                assert np.shape(result) == (24,), 'Expected a flat 24-value vector.'
                np.testing.assert_allclose(result, production.feature(x, base), atol=1e-12)
                np.testing.assert_array_equal(x, original)
                np.testing.assert_array_equal(base, original_base)
            assert calls, 'Call your moving_average from feature.'
        finally:
            namespace['moving_average'] = saved
        for value in (3., -3., 0.):
            x, base = np.full((19,12), value), np.zeros(12)
            np.testing.assert_allclose(function(x, base), production.feature(x, base), atol=1e-12)
    return _run_checks('feature', checks)


def load_frames(path):
    sequences, micros, values = [], [], []
    for number, line in enumerate(Path(path).read_text().splitlines(), 1):
        fields = line.strip().split(',')
        if fields[0] != 'FRAME':
            continue
        if len(fields) != 15:
            raise ValueError(f'{path}:{number}: expected 15 FRAME fields')
        seq, us = int(fields[1]), int(fields[2])
        row = np.asarray(fields[3:], dtype=float)
        if not (0 <= seq < 2**32 and 0 <= us < 2**32) or not np.isfinite(row).all():
            raise ValueError(f'{path}:{number}: invalid counters or sensor values')
        sequences.append(seq); micros.append(us); values.append(row)
    if len(values) < 2:
        raise ValueError(f'{path}: need at least two frames')
    ds = np.diff(np.array(sequences, dtype=np.int64)) % 2**32
    dt = (np.diff(np.array(micros, dtype=np.int64)) % 2**32) / 1e6
    if np.any((ds == 0) | (ds >= 2**31)) or np.any((dt <= 0) | (dt > .080)):
        raise ValueError(f'{path}: discontinuity or gap >80 ms; split/re-record trial')
    t = np.r_[0., np.cumsum(dt)]
    report = dict(frames=len(values), median_hz=float(1/np.median(dt)),
                  max_gap_ms=float(dt.max()*1000), missing_frames=int((ds-1).sum()))
    return t, np.stack(values), report


def load_trials(manifest):
    manifest = Path(manifest)
    with manifest.open(newline='') as handle:
        rows = list(csv.DictReader(handle))
    trials, ids, files = [], set(), set()
    for row in rows:
        path = (manifest.parent/row['file']).resolve()
        if not row['trial_id'] or row['trial_id'] in ids or path in files:
            raise ValueError('Every trial must have a unique ID and file.')
        ids.add(row['trial_id']); files.add(path)
        if row['label'] not in LABELS or row['split'] not in ('train','tune','test'):
            raise ValueError('Unknown label or split.')
        if not row['wearer_id'] or not row['session_id']:
            raise ValueError('Wearer and placement session required.')
        t, x, report = load_frames(path)
        a,b,c,d = (float(row[k]) for k in ('baseline_start_s','baseline_end_s','hold_start_s','hold_end_s'))
        if not (0 <= a < b <= c < d <= t[-1]) or b-a < 1 or d-c < .5:
            raise ValueError('Need in-bounds rest >=1s, separate hold >=0.5s.')
        grid = np.arange(int(np.floor(t[-1]*50))+1)/50
        uniform = np.column_stack([np.interp(grid,t,x[:,j]) for j in range(12)])
        mask = (grid >= a) & (grid < b)
        trials.append(dict(row=row,source='recorded',t=grid,x=uniform,raw_t=t,raw_x=x,report=report,
                           baseline=uniform[mask].mean(axis=0), rest=(a,b), hold=(c,d)))
        print(row['trial_id'], report)
    return trials


def _training_trial(trials, index):
    selected = [q for q in trials if q['row']['split']=='train' and q['row']['label']!='rest']
    if not selected:
        print('Need a non-rest training recording for this experiment.'); return None
    return selected[index]


def plot_smoothing(trials, average, index=0, channel=0):
    q = _training_trial(trials,index)
    if q is None: return
    y = average(q['x'])
    fig, ax = plt.subplots(figsize=(10,4))
    ax.plot(q['t'],q['x'][:,channel],alpha=.5,label='raw on uniform grid')
    ax.plot(q['t'][4:],y[:,channel],label='your five-tap average')
    ax.axvspan(*q['hold'],alpha=.1,color='orange',label='annotated hold')
    ax.set(xlabel='Device time (s)',ylabel=f'{CHANNELS[channel]} (µT)',title=f"{_source_name(q.get('source'))}: {q['row']['trial_id']}")
    ax.legend(); plt.show()


def plot_baseline_experiment(trials, feature, index=0):
    q = _training_trial(trials,index)
    if q is None: return
    c,d = q['hold']; t,x = q['t'],q['x']
    indices = np.flatnonzero((t>=c)&(t<d))
    if len(indices)<19: raise ValueError('Hold too short for 19 frames.')
    raw = x[indices[:19]]
    clean = feature(raw,q['baseline'])
    contaminated = feature(raw,x[indices].mean(axis=0))
    fig, axes = plt.subplots(2,1,figsize=(11,6),layout='constrained')
    for i,ax in enumerate(axes):
        ax.plot(CHANNELS,clean[12*i:12*i+12],'o-',label='clean rest baseline')
        ax.plot(CHANNELS,contaminated[12*i:12*i+12],'o-',label='held-pose baseline')
        ax.set_ylabel(['Signed mean (µT)','RMS (µT)'][i]); ax.legend()
    fig.suptitle(f"{_source_name(q.get('source'))}: {q['row']['trial_id']}")
    plt.show()


def build_examples(trials, feature):
    examples = []
    for q in trials:
        c,d = q['hold']
        for end in range(18,len(q['t']),5):
            if q['t'][end-18] >= c and q['t'][end] < d:
                examples.append(dict(trial_id=q['row']['trial_id'],split=q['row']['split'],
                                     label=q['row']['label'],source=q.get('source','recorded'),feature=feature(q['x'][end-18:end+1],q['baseline'])))
    print(len(examples),'held windows from',len({e['trial_id'] for e in examples}),'trials')
    return examples


def fit_training(examples):
    train = [e for e in examples if e['split']=='train']
    if not train or {e['label'] for e in train} != set(LABELS):
        print('Training pending: all four labels need training trials.'); return None
    model=production.make_model(np.stack([e['feature'] for e in train]),[e['label'] for e in train])
    model['lab_source']='synthetic' if all(e.get('source')=='synthetic' for e in train) else 'recorded'
    return model


def evaluate(examples, model, split, predict):
    items = [e for e in examples if e['split']==split]
    if model is None or not items:
        print(split,'evaluation pending.'); return None
    source='SYNTHETIC' if all(e.get('source')=='synthetic' for e in items) else 'Recorded'
    matrix = np.zeros((4,4),dtype=int); votes = {}
    for e in items:
        label = predict(model,e['feature'])
        matrix[LABELS.index(e['label']),LABELS.index(label)] += 1
        votes.setdefault(e['trial_id'],(e['label'],[]))[1].append(label)
    fig,ax = plt.subplots(figsize=(6,5)); ax.imshow(matrix,cmap='Blues')
    for (r,c),value in np.ndenumerate(matrix): ax.text(c,r,str(value),ha='center',va='center')
    ax.set(xticks=range(4),xticklabels=LABELS,yticks=range(4),yticklabels=LABELS,
           xlabel='Predicted',ylabel='Actual',title=f'{source} · {split}: window confusion counts')
    plt.show()
    correct = 0
    for tid,(actual,predictions) in votes.items():
        # Offline trial-vote count ties use the first encountered tied label.
        guess = Counter(predictions).most_common(1)[0][0]
        correct += guess==actual
        print(tid,'actual:',actual,'trial vote:',guess)
    print(f'{source} · {split}: trial votes correct {correct}/{len(votes)}; window accuracy {np.trace(matrix)/matrix.sum():.3f}')
    print('Per-class window support:',dict(zip(LABELS,matrix.sum(axis=1).tolist())))
    if np.any(matrix.sum(axis=1)==0): print('INCOMPLETE: missing class coverage.')
    return matrix


def replay(trials, model, feature, predict, index=0):
    choices = [q for q in trials if q['row']['split']=='tune' and q['row']['label']!='rest']
    if not choices: print('Replay pending: need a non-rest tuning trial.'); return
    q = choices[index]
    times,raw,stable = [],[],[]
    published,candidate,count = 'rest',None,0
    for end in range(18,len(q['t']),5):
        if q['t'][end-18] < q['rest'][1]: continue
        label = predict(model,feature(q['x'][end-18:end+1],q['baseline']))
        count = count+1 if candidate==label else 1
        candidate = label
        if count>=2: published = label
        times.append(q['t'][end]); raw.append(LABELS.index(label)); stable.append(LABELS.index(published))
    fig,ax=plt.subplots(figsize=(10,4))
    ax.step(times,raw,where='post',alpha=.5,label='raw')
    ax.step(times,stable,where='post',label='two matching predictions')
    ax.axvspan(*q['hold'],alpha=.1,color='orange',label='annotated hold')
    ax.set(yticks=range(4),yticklabels=LABELS,xlabel='Device time (s)',title=f"{_source_name(q.get('source'))}: {q['row']['trial_id']}")
    ax.legend(); plt.show()


def _visual_trial(trials, index):
    """Use real training data when available; otherwise use an explicit illustration."""
    choices = [q for q in trials if q['row']['split'] == 'train' and q['row']['label'] != 'rest']
    if choices:
        q = choices[index]
        return q, f"{_source_name(q.get('source'))}: {q['row']['trial_id']}"
    t = np.arange(350)/50
    baseline = np.linspace(100., 210., 12)
    displacement = np.array([8., -5., 3., -7., 4., 2., 6., -3., 5., -4., 7., -2.])
    envelope = np.clip((t-2.)/.4, 0, 1)*np.clip((5.4-t)/.4, 0, 1)
    noise = .6*np.sin(2*np.pi*11*t[:,None] + np.arange(12)[None,:])
    x = baseline + envelope[:,None]*displacement + noise
    return dict(t=t, x=x, baseline=baseline, rest=(0.,1.5), hold=(3.,5.)), 'ILLUSTRATION ONLY · no wearer recording'


def _visual_output(function, *args):
    try:
        return np.asarray(function(*args), dtype=float), None
    except NotImplementedError:
        return None, 'Complete the function, then rerun this cell.'
    except Exception as exc:
        return None, f'{type(exc).__name__}: {exc}'


def _pending_axis(ax, message):
    ax.text(.5,.5,message,transform=ax.transAxes,ha='center',va='center',wrap=True)
    ax.set_xticks([]); ax.set_yticks([])


def plot_average_steps(trials, average, index=0, channel=0, window_start_s=None):
    q, title = _visual_trial(trials,index)
    t,x = q['t'],q['x']
    if not isinstance(channel, (int, np.integer)) or not 0 <= channel < 12:
        raise ValueError('CHANNEL must be an integer from 0 to 11.')
    requested = q['hold'][0] if window_start_s is None else float(window_start_s)
    if not np.isfinite(requested) or not t[0] <= requested <= t[-19] + 1e-9:
        raise ValueError(f'WINDOW_START_S must be between {t[0]:.2f} and {t[-19]:.2f} seconds.')
    start = min(int(np.searchsorted(t, requested)), len(t)-19)
    raw, times = x[start:start+19],t[start:start+19]
    y,error = _visual_output(average,raw)
    if y is not None and y.shape != (15,12):
        error = f'Your output shape is {y.shape}; expected (15, 12).'; y=None
    fig,axes = plt.subplots(2,2,figsize=(12,7),layout='constrained')
    ax=axes[0,0]
    ax.plot(t,x[:,channel],alpha=.65,label='input')
    ax.axvspan(times[0],times[-1],alpha=.2,color='orange',label='selected 19 rows')
    ax.set(title=f'{CHANNELS[channel]} · full input',xlabel='Device time (s)',ylabel='µT'); ax.legend()
    ax=axes[0,1]
    ax.plot(times,raw[:,channel],'o-',alpha=.5,label='19 input samples')
    ax.axvspan(times[0],times[4],alpha=.15,color='green',label='first five inputs')
    if y is not None: ax.plot(times[4:],y[:,channel],'o-',label='your 15 outputs')
    else: ax.text(.03,.97,error,transform=ax.transAxes,va='top',fontsize=9,wrap=True)
    ax.set(title=f'{CHANNELS[channel]} · {times[0]:.2f}–{times[-1]:.2f} s',xlabel='Device time (s)',ylabel='µT');ax.legend(fontsize=8)
    raw_change = raw - q['baseline']
    smooth_change = None if y is None else y - q['baseline']
    span = max(float(np.max(np.abs(raw_change))),
               0. if smooth_change is None else float(np.max(np.abs(smooth_change))), 1e-9)
    for ax,values,label in [(axes[1,0],raw_change,'Input change from rest · 19 × 12'),
                            (axes[1,1],smooth_change,'Smoothed change from rest · 15 × 12')]:
        ax.set_title(label)
        if values is None:
            _pending_axis(ax,error); continue
        im=ax.imshow(values,aspect='auto',vmin=-span,vmax=span,cmap='RdBu_r')
        ax.add_patch(Rectangle((channel-.5,-.5),1,len(values),fill=False,edgecolor='black',linewidth=2.5))
        ax.set(xticks=range(12),xticklabels=CHANNELS,xlabel='Channel',ylabel='Output row' if ax is axes[1,1] else 'Input row')
        ax.tick_params(axis='x',labelrotation=45);fig.colorbar(im,ax=ax,label='Change from rest (µT)')
    fig.suptitle(f'{title}\nCHANNEL={channel} ({CHANNELS[channel]}) · window={times[0]:.2f}–{times[-1]:.2f} s')
    plt.show()
    return fig


def plot_feature_steps(trials, average, feature, index=0, channel=0):
    q,title = _visual_trial(trials,index)
    start=int(np.searchsorted(q['t'],q['hold'][0]))
    raw=q['x'][start:start+19]; times=q['t'][start+4:start+19];base=q['baseline']
    y,error = _visual_output(average,raw)
    f,feature_error = _visual_output(feature,raw,base)
    if y is not None and y.shape != (15,12):error=f'Expected (15,12), received {y.shape}';y=None
    if f is not None and f.shape != (24,):feature_error=f'Expected (24,), received {f.shape}';f=None
    fig,axes=plt.subplots(3,1,figsize=(11,9),layout='constrained')
    axes[0].set_title(f'{CHANNELS[channel]} · smoothing and relaxed reference')
    axes[0].plot(q['t'][start:start+19],raw[:,channel],'o-',alpha=.4,label='raw')
    axes[0].axhline(base[channel],color='gray',ls='--',label='relaxed baseline')
    if y is not None:axes[0].plot(times,y[:,channel],'o-',label='your smoothed values')
    axes[0].set(xlabel='Device time (s)',ylabel='µT');axes[0].legend()
    axes[1].set_title('Baseline-relative signal and your summaries')
    if y is None:_pending_axis(axes[1],error)
    else:
        delta=y[:,channel]-base[channel]  # display only; classification calls student's feature
        axes[1].plot(times,delta,'o-',label='your smoothed signal − baseline')
        axes[1].axhline(0,color='gray',lw=.8)
        if f is not None:
            axes[1].axhline(f[channel],color='tab:orange',label='your signed mean')
            axes[1].axhline(f[12+channel],color='tab:green',ls='--',label='your RMS')
        else:axes[1].text(.02,.95,feature_error,transform=axes[1].transAxes,va='top')
        axes[1].set(xlabel='Device time (s)',ylabel='Change (µT)');axes[1].legend()
    axes[2].set_title('Your 24 features · 12 signed means, then 12 RMS values')
    if f is None:_pending_axis(axes[2],feature_error)
    else:
        axes[2].bar(np.arange(12),f[:12],color='tab:orange',label='signed mean')
        axes[2].bar(np.arange(12,24),f[12:],color='tab:green',label='RMS')
        axes[2].axvline(11.5,color='gray',ls='--');axes[2].axhline(0,color='gray',lw=.8)
        axes[2].set(xticks=range(24),xticklabels=CHANNELS+CHANNELS,xlabel='Feature order',ylabel='µT')
        axes[2].tick_params(axis='x',labelrotation=60);axes[2].legend()
    fig.suptitle(title)
    plt.show()
    return fig


def verify_distances(function):
    def checks():
        rng=np.random.default_rng(54)
        for size in (4,13):
            features=rng.normal(size=(size,24))*np.linspace(.1,15,24)
            labels=[LABELS[i%4] for i in range(size)]
            model=production.make_model(features,labels)
            for sample in (features[0],rng.normal(size=24)):
                output=function(model,sample)
                assert np.shape(output)==(size,), 'Return one distance per training example.'
                np.testing.assert_allclose(output,production.normalized_distances(model,sample),atol=1e-12)
    return _run_checks('normalized_distances',checks)


def verify_predict(function, distances, distances_ready):
    if not distances_ready:
        print('predict: PENDING — finish normalized_distances first.'); return False
    def checks():
        namespace=function.__globals__;saved=namespace.get('normalized_distances');calls=[]
        def tracked(model,sample):
            calls.append(True);return distances(model,sample)
        namespace['normalized_distances']=tracked
        try:
            rng=np.random.default_rng(55)
            features=rng.normal(size=(16,24))
            model=production.make_model(features,LABELS*4)
            for sample in rng.normal(size=(20,24)):
                assert function(model,sample)==production.predict(model,sample), 'Wrong label.'
            # Exercise majority, three-way vote ties, and equal-distance ordering.
            for positions,labels in [([1,2,3,9],['rest','fist','fist','spread']),
                                     ([3,1,2,9],['rest','fist','spread','wrist_up']),
                                     ([1,-1,1,9],['spread','rest','fist','wrist_up'])]:
                features=np.zeros((4,24));features[:,0]=positions
                model=production.make_model(features,LABELS)
                model['labels']=labels
                model['feature_mean']=[0.]*24;model['feature_scale']=[1.]*24
                sample=np.zeros(24)
                assert function(model,sample)==production.predict(model,sample), 'Check voting and tie rules.'
            assert calls, 'Call your normalized_distances from predict.'
        finally:
            namespace['normalized_distances']=saved
    return _run_checks('predict',checks)


def plot_knn_steps(distances, predict, model=None, query_row=0, axes=(0,1)):
    """Plot student distances, selected neighbors, and student vote; never substitute predictions."""
    if model is None:
        rng=np.random.default_rng(56)
        model=production.make_model(rng.normal(size=(16,24)),LABELS*4)
        title='ILLUSTRATIVE FEATURE VECTORS · complete features to use the recordings'
    else:
        title=f"{_source_name(model.get('lab_source'))} training examples · illustrative query, not evaluation"
    features=np.asarray(model['features'],dtype=float)
    scale=np.asarray(model['feature_scale'],dtype=float)
    mean=np.asarray(model['feature_mean'],dtype=float)
    # A deliberately constructed query; not a held-out accuracy measurement.
    query=features[query_row]+np.linspace(-.1,.1,24)*scale
    z=(features-mean)/scale; query_z=(query-mean)/scale
    result,error=_visual_output(distances,model,query)
    if result is not None and (result.shape!=(len(features),) or not np.isfinite(result).all() or np.any(result<0)):
        error='Distances must be finite, nonnegative, and have shape (N,).';result=None
    fig,panels=plt.subplots(1,3,figsize=(14,4.5),layout='constrained')
    a,b=axes
    for label in LABELS:
        mask=np.asarray(model['labels'])==label
        panels[0].scatter(z[mask,a],z[mask,b],s=22,alpha=.5,label=label)
    panels[0].scatter(query_z[a],query_z[b],marker='*',s=180,color='black',label='query')
    panels[0].set(title='Two-feature view · distance uses all 24',xlabel=f'Scaled feature {a}',ylabel=f'Scaled feature {b}')
    panels[0].legend(fontsize=8)
    panels[1].set_title('Your distances · nearest examples')
    panels[2].set_title('Vote among your three nearest')
    if result is None:
        _pending_axis(panels[1],error);_pending_axis(panels[2],error)
    else:
        order=np.argsort(result,kind='stable');nearest=order[:int(model['k'])];shown=order[:10]
        panels[0].scatter(z[nearest,a],z[nearest,b],s=110,facecolors='none',edgecolors='black',linewidths=1.5)
        panels[1].bar(range(len(shown)),result[shown],color=['tab:orange' if i in nearest else 'lightgray' for i in shown])
        panels[1].set(xticks=range(len(shown)),xticklabels=[f"{i}: {model['labels'][int(i)]}" for i in shown],ylabel='Your standardized distance')
        panels[1].tick_params(axis='x',labelrotation=60)
        counts=Counter(model['labels'][int(i)] for i in nearest)
        panels[2].bar(LABELS,[counts[label] for label in LABELS]);panels[2].set(ylabel='Votes',yticks=range(4))
        try:
            label=predict(model,query)
            panels[2].set_title(f'Your prediction: {label}')
        except NotImplementedError:
            panels[2].set_title('Votes shown · implement predict')
        except Exception as exc:
            panels[2].set_title(f'Prediction error: {type(exc).__name__}')
    fig.suptitle(title);plt.show()
    return fig


def make_synthetic_trials(seed=90393):
    """Create independent, labeled 50 Hz magnetic trials for teaching, not hardware evidence."""
    rng=np.random.default_rng(seed)
    t=np.arange(350)/50
    rest=(0.,1.5);hold=(3.,5.)
    baseline=np.array([330.,-420.,180.,280.,-350.,130.,190.,-270.,210.,320.,-190.,150.])
    patterns=np.array([
        [0,0,0,0,0,0,0,0,0,0,0,0],
        [9,3,-5,7,2,-4,4,-3,2,3,-2,4],
        [3,10,-6,-4,8,-5,8,5,-3,-7,4,2],
        [-10,-6,7,-8,-9,6,-5,-7,10,9,-4,-6],
    ],dtype=float)
    trials=[]
    for split,count in [('train',6),('tune',2),('test',2)]:
        for repeat in range(count):
            for class_index in rng.permutation(4):
                label=LABELS[class_index]
                onset=2.15+rng.uniform(-.08,.08)
                rise=np.clip((t-onset)/rng.uniform(.25,.55),0,1)
                release=np.clip((5.15+rng.uniform(-.05,.1)-t)/rng.uniform(.25,.5),0,1)
                envelope=rise*release if label!='rest' else np.zeros_like(t)
                offset=baseline+rng.normal(0,4,12)
                response=patterns[class_index]*rng.normal(1.,.13)+rng.normal(0,.6,12)
                noise=rng.normal(0,.8,(350,12))
                ripple=.55*np.sin(2*np.pi*rng.uniform(9,13)*t[:,None]+rng.uniform(0,2*np.pi,12))
                drift=np.sin(2*np.pi*.12*t[:,None])*rng.normal(0,.4,12)
                x=offset+envelope[:,None]*response+noise+ripple+drift
                tid=f'synthetic_{split}_{label}_{repeat+1:02d}'
                trials.append(dict(row=dict(trial_id=tid,label=label,split=split,
                                             wearer_id='synthetic',session_id='synthetic'),
                                   source='synthetic',t=t.copy(),x=x,raw_t=t.copy(),raw_x=x.copy(),
                                   baseline=x[(t>=rest[0])&(t<rest[1])].mean(axis=0),
                                   rest=rest,hold=hold,
                                   report=dict(frames=350,median_hz=50.,max_gap_ms=20.,missing_frames=0)))
    return trials


def load_synthetic_trials(path):
    """Load bundled numeric recordings without pickle or hardware access."""
    with np.load(path,allow_pickle=False) as data:
        t=data['time_s'].copy();signals=data['signals_ut'].copy()
        labels=data['labels'].tolist();splits=data['splits'].tolist();ids=data['trial_ids'].tolist()
        rest=tuple(data['rest_interval_s'].tolist());hold=tuple(data['hold_interval_s'].tolist())
    if signals.shape!=(40,350,12) or t.shape!=(350,) or not np.isfinite(signals).all():
        raise ValueError('Invalid synthetic recording dimensions or values.')
    if not np.allclose(t,np.arange(350)/50) or len(set(ids))!=40:
        raise ValueError('Invalid synthetic times or duplicate trial IDs.')
    trials=[]
    for x,label,split,tid in zip(signals,labels,splits,ids):
        if label not in LABELS or split not in ('train','tune','test'):
            raise ValueError('Invalid synthetic label or split.')
        trials.append(dict(row=dict(trial_id=tid,label=label,split=split,
                                    wearer_id='synthetic',session_id='synthetic'),
                           source='synthetic',t=t.copy(),x=x,raw_t=t.copy(),raw_x=x.copy(),
                           baseline=x[(t>=rest[0])&(t<rest[1])].mean(axis=0),rest=rest,hold=hold,
                           report=dict(frames=350,median_hz=50.,max_gap_ms=20.,missing_frames=0)))
    return trials


def _source_name(source):
    return 'SYNTHETIC' if source=='synthetic' else 'Recording'
