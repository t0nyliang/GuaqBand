# Four-Sensor Finger Movement Armband

This project reads four MLX90393 magnetic sensors through an ESP32 and
recognizes `rest`, `wrist_up`, `spread`, and `fist`. The first-run path is:

1. Upload the ESP32 firmware.
2. Find the ESP32 serial port.
3. Run guided calibration for the person wearing the armband.
4. Start live detection or the monitoring dashboard.

Four MLX90393 sensors connect to an ESP32 through PCA9548 channels 0, 2, 5, and
7. Because each sensor is isolated by the mux, all four can use their default
I2C address of `0x18`. The firmware reads them in that channel order and emits
one timestamped frame at a target rate of 50 Hz:

```text
FRAME,sequence,device_us,s0x,s0y,s0z,s1x,s1y,s1z,s2x,s2y,s2z,s3x,s3y,s3z
```

The mux reads are sequential, so the four measurements are grouped into one
near-synchronous frame rather than captured at exactly the same instant.

Picture of the GuaqBand on a forearm:
<img width="2160" height="2880" alt="image" src="https://github.com/user-attachments/assets/940a4efa-e04d-4e30-89ec-c344605cf1a1" />


## First-time setup

### 1. Upload firmware

In Arduino IDE, open and upload
[`mlx90393_live/mlx90393_live.ino`](mlx90393_live/mlx90393_live.ino). Select
the correct ESP32 board and port under **Tools** before uploading. The combined
[BNO085 + MLX90393 firmware](motion_pipeline/firmware/bno085_uart_rvc/bno085_uart_rvc.ino)
also works: it emits the same `FRAME` packets plus motion packets.

After uploading, **close Arduino Serial Monitor and Serial Plotter**. Only one
program can use the ESP32 serial port at a time.

### 2. Create the calibration environment

From the repository root:

```powershell
cd .\calibration_pipeline
python -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
```

On macOS or Linux, use the equivalent `.venv/bin/python` path.

### 3. Find the ESP32 port

Use the port belonging to the ESP32, not a Bluetooth or unrelated USB device.

- **Windows:** Select **Tools > Port** in Arduino IDE, or look under **Ports
  (COM & LPT)** in Device Manager. A label such as `CP210x (COM3)` means the
  port value is `COM3`.
- **macOS:** Run `ls /dev/cu.usb*` and use the new device path, such as
  `/dev/cu.usbserial-110`.
- **Linux:** Run the port-listing command below, or inspect `/dev/ttyUSB*` and
  `/dev/ttyACM*`. Typical values are `/dev/ttyUSB0` and `/dev/ttyACM0`. If
  access is denied, add your account to `dialout`, then sign out and back in.

From `calibration_pipeline`, this cross-platform command lists each detected
port with its USB description:

```powershell
.\.venv\Scripts\python.exe -m serial.tools.list_ports -v
```

Replace `COM3` in the commands below with the port you found.

## Optional live plot

From the repository root, upload `mlx90393_live/mlx90393_live.ino`, close
Arduino Serial Monitor, and run:

```powershell
python -m pip install -r .\mlx90393_live\requirements.txt
python .\mlx90393_live\plot_live.py --port COM3
```

The default 2x2 dashboard shows Bx, By, and Bz for all four sensors. To focus on
one panel:

```powershell
python .\mlx90393_live\plot_live.py --port COM3 --sensor 2
```

If a sensor cannot be read, its values are sent as `nan` while the remaining
sensors continue updating. The firmware retries unavailable sensors once per
second. The plot also reports sequence gaps so serial data loss is visible.

To read the BNO085 and all four MLX90393 sensors from one ESP32, upload
`motion_pipeline/firmware/bno085_uart_rvc.ino` instead. It emits the same
`FRAME` packets plus `MOTION` packets; the existing plot ignores the motion
packets.

Legacy `SAMPLE` and `DATA` packets are still accepted, but only Sensor 0 will
contain data when using those formats.

## Calibrate the armband

Calibration creates a personal model at `calibration_pipeline/profile.json`.
Re-run it whenever the wearer, sensor placement, or armband fit changes. The
pipeline requires all four sensors and consumes all 12 values in each `FRAME`.

```powershell
cd .\calibration_pipeline
python -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
.\.venv\Scripts\python.exe -m eflesh_calibration calibrate --port COM3
.\.venv\Scripts\python.exe -m eflesh_calibration recalibrate --port COM3 --gesture spread
.\.venv\Scripts\python.exe -m eflesh_calibration live --port COM3
.\.venv\Scripts\python.exe -m eflesh_calibration monitor --port COM3
```

Keep the armband in the same position throughout. The guided program records a
relaxed baseline, then captures 10 two-second examples each of `rest`,
`wrist_up`, `spread`, and `fist` (40 recordings total). Press Enter when
prompted. At each `GO`, make the requested pose and hold it until `captured`
appears. For `rest`, stay relaxed; for the other gestures, begin moving at `GO`
and hold the final position. The completed calibration saves automatically.

The `recalibrate` command replaces only the selected gesture's examples. Valid
gesture values are `rest`, `wrist_up`, `spread`, and `fist`.

Before `live` or `monitor`, close every other serial tool and keep your hand
relaxed while the fresh baseline is collected. Live mode publishes a label
after two matching predictions; `ONSET` marks a newly detected non-rest
gesture.

See [calibration_pipeline/README.md](calibration_pipeline/README.md) for the
detector details.

`monitor` opens a desktop dashboard with the stabilized label and the existing
per-gesture KNN proximity scores as relative likelihood bars. It does not alter
the classifier or calibration data.
