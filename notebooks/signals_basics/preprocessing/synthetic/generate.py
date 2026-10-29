"""Rebuild the bundled synthetic recordings; run with the shared Python environment."""
from pathlib import Path
import sys
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[3]))
sys.path.insert(0, str(HERE.parent))
from lab_support import make_synthetic_trials


def main():
    trials = make_synthetic_trials(seed=90393)
    destination = HERE / 'gesture_trials.npz'
    np.savez_compressed(
        destination,
        time_s=trials[0]['t'], signals_ut=np.stack([q['x'] for q in trials]),
        labels=np.array([q['row']['label'] for q in trials]),
        splits=np.array([q['row']['split'] for q in trials]),
        trial_ids=np.array([q['row']['trial_id'] for q in trials]),
        rest_interval_s=np.array(trials[0]['rest']), hold_interval_s=np.array(trials[0]['hold']),
        seed=np.array(90393), source=np.array('synthetic'), sample_hz=np.array(50), units=np.array('µT'),
    )
    print(destination)


if __name__ == '__main__':
    main()
