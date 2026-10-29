# Raw recording placeholder

The notebook runs on bundled synthetic trials by default. To switch to real data, set `USE_SYNTHETIC = False`, put raw FRAME text logs here and add one row per independent trial to `manifest.csv`. Leave the header intact. An example row, to add only after recording the actual file:

```csv
p01_s01_fist_01.txt,p01,s01,p01_s01_fist_01,fist,train,0,1.5,3,5
```

Columns: file, wearer_id, session_id, trial_id, label, split, baseline_start_s, baseline_end_s, hold_start_s, hold_end_s. File paths are relative to this folder. IDs and files must be unique per trial. Use pseudonymous wearer IDs. Reseating the band starts a new placement session.

Use existing MLX firmware or combined firmware, retaining lines formatted as:

```text
FRAME,sequence,device_us,s0x,s0y,s0z,s1x,s1y,s1z,s2x,s2y,s2z,s3x,s3y,s3z
```

The field names above describe the format; actual lines contain numbers after FRAME. Sensor values are µT. Sensor order follows mux channels 0, 2, 5, 7. BNO085 MOTION packets are ignored for this lab. Serial capture is already demonstrated in the existing build notebook; use any logger that preserves complete raw lines. Close other serial readers before recording.

Collect 10 trials per label: rest, wrist_up, spread, fist. Assign six train, two tune, two test before analysis. Interleave label order within each block. Each seven-second recording has 2 s rest, 1 s movement, 2 s hold, 2 s release/rest. For rest trials remain relaxed. Times in the manifest are seconds from the first device frame; inspect and correct actual intervals. Suggested rest [0,1.5) and hold [3,5) are starting annotations, not guaranteed ground truth.

The loader requires at least 1 s of known rest and at least 0.5 s of separate held data. It reports cadence and missing frames, supports uint32 wrap, rejects nonfinite values and counter reversals, and rejects gaps above 80 ms. That teaching cutoff differs from the production live policy. Short gaps are interpolated; inspect the report before accepting a trial. Re-record or split invalid files rather than inventing values.

Never split overlapping windows from one recording across datasets. For new-session or new-wearer evaluation, hold out whole sessions/wearers. Do not tune using test results. A rest calibration before each trial is part of the protocol.
