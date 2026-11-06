# Live hardware lab: student instructions

Download [the student kit](guaqband-live-integration.zip), then follow these steps.

This kit supplies the serial reader, timed windows, guided calibration, model storage, and live label stabilization. Your task is to paste your four completed notebook functions into `calibration_pipeline/eflesh_calibration/knn.py`. They are intentionally blank here. Bring your own completed notebook; no solutions or personal calibration profile are included.

## 1. Use your own branch and a separate pipeline

Use your own local clone of the class repository. You do not need a new repository. From its root, create your branch, replacing `alex` with your name:

```bash
git status
git switch -c students/alex/live-lab
```

If you already have your own branch, continue on it. Existing uncommitted notebook work follows you onto the new branch; commit your completed notebook before integration.

Extract `guaqband-live-integration.zip`. Rename the extracted `guaqband-live-integration-student` folder to **`live_lab`** and put it inside your repository:

```text
GuaqBand/
├── calibration_pipeline/       # Existing instructor implementation
├── notebooks/                  # Your notebook work
└── live_lab/                    # Your separate runnable pipeline
    ├── calibration_pipeline/
    ├── mlx90393_live/
    ├── check_integration.py
    ├── requirements.txt
    └── AI_PROMPT.md
```

Keep the instructor's original `calibration_pipeline/` unchanged. Your algorithms belong in **`live_lab/calibration_pipeline/eflesh_calibration/knn.py`**. Do not copy the kit over the original pipeline.

Open `live_lab/` in your editor, and run Python commands from that folder:

```bash
cd live_lab
```

Check that `check_integration.py` is directly inside this folder, not inside another nested ZIP folder. Running from the repository root would select the instructor's pipeline instead of yours.

The important files:

| File | Purpose |
|---|---|
| `calibration_pipeline/eflesh_calibration/knn.py` | Paste your four functions; keep its supplied model helpers/constants |
| `sensor.py` in that package | Read and parse ESP32 serial frames |
| `app.py` in that package | Calibration, resampling, rolling windows, stable labels and gesture events |
| `monitor.py` in that package | Optional Tk desktop dashboard |
| `mlx90393_live/mlx90393_live.ino` | ESP32 firmware |
| `check_integration.py` | Offline checks using synthetic recordings; no hardware needed |
| `AI_PROMPT.md` | Prompt for connecting your own notebook implementations |

## 2. Python setup

Use Python 3.10 or newer. On macOS/Linux:

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
```

On Windows PowerShell:

```powershell
python -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install -r requirements.txt
```

If activation is unavailable, replace `python` below with `.\.venv\Scripts\python.exe` (Windows) or `./.venv/bin/python` (macOS/Linux).

## 3. Connect your functions

Give your AI coding tool:

- The extracted `live_lab/` folder as its working project.
- Your completed notebook from `notebooks/signals_basics/preprocessing/preprocessing_lab.ipynb` (attach it or grant the tool access).
- The prompt in `AI_PROMPT.md`.

Ask it to transfer your `moving_average`, `feature`, `normalized_distances`, and `predict` functions into the kit. Reuse your implementations; this is integration, not a second implementation exercise. Keep the supplied model helpers, serial reader, timestamp handling, calibration and label stabilization. If imports are needed, add them inside the student pipeline. Have the AI explain any corrections and read its changes before testing.

Do not let it modify the original repository pipeline or use the instructor's completed algorithms in place of your notebook work.

After integrating, run:

```bash
python check_integration.py
python -m calibration_pipeline.eflesh_calibration --help
```

An untouched kit reports missing implementations. The offline check reads synthetic data, uses only temporary test profiles, and does not access serial hardware or a real profile. All four function checks should pass before connecting hardware.

## 4. Firmware and hardware

In Arduino IDE, install the ESP32 board support and the **Adafruit MLX90393** library, accepting its dependencies. Open `mlx90393_live/mlx90393_live.ino`, select the ESP32 board and port, and upload it. If the correct firmware is already installed, keep it.

Hardware: ESP32 SDA=21 and SCL=22, PCA9548 mux at 0x70, four MLX90393 sensors at 0x18 on mux channels 0, 2, 5, 7. Serial speed is 115200 baud. The target is 50 Hz; all four sensors must provide finite XYZ readings. Packets are `FRAME,sequence,device_us,s0x,...,s3z` (12 numeric channels).

Close Arduino Serial Monitor/Plotter and any other serial reader before running Python.

## 5. Calibrate and run

Find your port:

```bash
python -m serial.tools.list_ports -v
```

Replace `COM3` below with the ESP32 port; macOS may look like `/dev/cu.usbserial-110` and Linux like `/dev/ttyUSB0`.

```bash
python -m calibration_pipeline.eflesh_calibration calibrate --port COM3
python -m calibration_pipeline.eflesh_calibration live --port COM3
```

Run those commands separately. Calibration starts with two seconds of relaxed data, then captures ten two-second trials each of rest, wrist_up, spread, and fist. Follow the countdowns: stay relaxed for rest, otherwise move at GO and hold until captured. Keep the band position fixed.

Calibration creates your local `calibration_pipeline/profile.json`. Live mode collects a fresh relaxed baseline and then prints stable predictions and new-gesture events. Recalibrate for a new wearer or placement. Keep profiles local.

Optional dashboard, if Python has Tk installed:

```bash
python -m calibration_pipeline.eflesh_calibration monitor --port COM3
```

## Done when

Your four functions pass the offline checks, calibration completes on your armband, and you can demonstrate rest plus a deliberate gesture and release. Explain where your functions run and distinguish an offline check from a real hardware test. If hardware is unavailable, the synthetic check lets you finish integration; label the hardware demo as pending.

## 6. Commit and submit

From `live_lab/`, return to the repository root and review your changes:

```bash
cd ..
git status
git diff -- calibration_pipeline
```

The final command should show no changes to the original pipeline. Review the new files in `live_lab/` too, then commit and push your own branch:

```bash
git add live_lab
git commit -m "Integrate notebook algorithms with live hardware"
git push -u origin students/alex/live-lab
```

Use your actual branch name. The kit's `.gitignore` excludes its Python environment and calibration profile. Do not force-add those files.

Submit your branch link with:

- The offline check results.
- A live demonstration of rest, a deliberate gesture, and release (or hardware marked pending).
- An explanation of how one sensor window becomes a label and one failure or limitation you observed.

Leave merging into `main` to the instructor.
