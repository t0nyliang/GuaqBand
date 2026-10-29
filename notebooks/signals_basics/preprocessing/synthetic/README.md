# Synthetic recordings

`gesture_trials.npz` contains 40 generated seven-second trials at 50 Hz, with 12 magnetic-field channels in µT. Each gesture (`rest`, `wrist_up`, `spread`, `fist`) has six training, two tuning and two test trials. Entire trials stay in one split.

The modeled sequence is rest, movement starting around 2.15 s, hold from 3–5 s, release, then rest. Rest-labeled trials remain relaxed. Channels have fixed illustrative gesture patterns with independent trial variations in baseline, amplitude, timing, noise, high-frequency ripple and slow drift. These are teaching signals, not recordings from a wearer or a physical magnetic-field simulation.

The notebook uses this file by default. The current lab uses these trials for preprocessing and KNN visualization. Tuning/test trials remain available for future evaluation; evaluation and replay are outside this lab. Synthetic scores do not establish hardware or wearer accuracy. Query examples in the neighbor visualization are constructed from training features, not held-out evaluation queries.

Contents: `time_s` `(350,)`, `signals_ut` `(40,350,12)`, trial labels/splits/IDs, rest/hold intervals, seed, source, units and sample rate. The arrays load without pickle.

To reproduce the file from the repository root:

```bash
python notebooks/signals_basics/preprocessing/synthetic/generate.py
```

The deterministic seed is 90393. The generator is `make_synthetic_trials` in `lab_support.py`. The generator replaces only this synthetic dataset, never a real recording.
