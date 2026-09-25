# Minimal Four-Sensor Calibration

This is a deliberately small teaching pipeline for recognizing `wrist_up`,
`spread`, and `fist` with four MLX90393 sensors. It favors readable code over
production accuracy.

The ESP32 must send 50 Hz frames with twelve XYZ values:

```text
FRAME,sequence,device_us,s0x,s0y,s0z,...,s3x,s3y,s3z
```

## How it works

1. Average two seconds of relaxed sensor data for a baseline.
2. Capture ten two-second recordings of `rest`, `wrist_up`, `spread`, and
   `fist`.
3. Resample each timed recording to 50 Hz, trim half a second from each edge,
   and extract eight overlapping 300 ms windows from its clean one-second
   center.
4. Apply a causal five-sample moving average, then subtract the baseline and
   calculate a signed mean and RMS magnitude for each of the 12 channels.
5. Save the resulting 320 labeled 24-value feature vectors and their
   normalization statistics to `profile.json`.
6. Use ESP32 timestamps to resample a rolling 360 ms live history to 50 Hz,
   standardize its features, and classify it using the three nearest examples.
   Publish a new label after two consecutive matching predictions.

There is no threshold tuning, validation trial, replay, or model fitting beyond
storing examples for KNN.

The implementation has three main files:

- `sensor.py` parses frames and reads the serial port.
- `knn.py` creates RMS features and performs nearest-neighbor voting.
- `app.py` provides calibration, live use, and the command line.

## Setup

Upload `..\mlx90393_live\mlx90393_live.ino` in Arduino IDE, then close Arduino
Serial Monitor and Serial Plotter. The combined BNO085 firmware also works
because it emits the same `FRAME` packets. Only one program can use the serial
port at a time.

Install the runtime dependencies from this directory:

```powershell
cd .\calibration_pipeline
python -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
```

Find the ESP32 port before running a command:

```powershell
.\.venv\Scripts\python.exe -m serial.tools.list_ports -v
```

On Windows, it is usually the `COM` number displayed in Arduino IDE under
**Tools > Port** or in Device Manager under **Ports (COM & LPT)**. On macOS it
is typically `/dev/cu.usbserial-*`; on Linux it is usually `/dev/ttyUSB0` or
`/dev/ttyACM0`. Replace `COM3` below with that value.

## Calibrate

```powershell
.\.venv\Scripts\python.exe -m eflesh_calibration calibrate --port COM3
```

Keep the armband position fixed. The guide first records a relaxed baseline,
then captures each class in order. Press Enter once before each set and follow
the three-second countdowns. Frames buffered during each countdown are
discarded at `GO`, and capture stops using a wall-clock timer after two seconds.
The received frames are resampled to 50 Hz before feature extraction, so a
slower hardware frame rate cannot make the gesture hold run long.
For `wrist_up`, flex your wrist upward after `GO` and hold it through the
two-second recording. The program prints
`capturing for 2 seconds...` while it records. The first and last half-second
are excluded from training windows, leaving one second for the complete
movement and held position and making the capture less sensitive to movement
timing. Calibration always saves after all 40 recordings.

At `GO`, remain relaxed for `rest`; for the other gestures, make the indicated
movement and hold its final position until `captured` is printed. Re-run the
full calibration when the wearer, armband fit, or sensor placement changes.

## Recalibrate one gesture

To replace only one gesture's saved examples, keep the armband in the same
position and run:

```powershell
.\.venv\Scripts\python.exe -m eflesh_calibration recalibrate --port COM3 --gesture spread
```

Valid gestures are `rest`, `wrist_up`, `spread`, and `fist`. The guide records
a fresh relaxed baseline and ten examples of only the selected gesture. It
keeps the other three gesture sets, replaces the selected set in
`profile.json`, and recalculates the model's normalization statistics.

## Run live

```powershell
.\.venv\Scripts\python.exe -m eflesh_calibration live --port COM3
```

Live mode takes a fresh relaxed baseline and prints moving-average-filtered,
normalized predictions once per physical sensor frame. Each prediction uses a
timestamp-resampled 360 ms history, so slower hardware does not stretch the
detector window. A label must win two consecutive predictions before it is
published. `ONSET` is printed when a new stable non-rest gesture appears.

## Desktop monitoring dashboard

Use the monitor command to view the existing detector in a desktop window
instead of the terminal:

```powershell
.\.venv\Scripts\python.exe -m eflesh_calibration monitor --port COM3
```

It shows the stabilized live label and a bar for each gesture. The bars are the
existing KNN class-proximity display scores normalized to 100%, so they are
useful for comparing relative confidence but are not calibrated probabilities.

Profiles from the older pipeline are intentionally unsupported. Run calibration
again if the profile format error appears.

## Test

```powershell
.\.venv\Scripts\python.exe -m pytest tests -q
```
