# Magnetic sensor firmware

Connect the four MLX90393 sensors to PCA9548 mux channels **0, 1, 2, and 3**.
Both this sketch and the combined BNO085 sketch use that channel order.

## Upload with VS Code or Cursor

1. Install the **PlatformIO IDE** extension and open the repository root
   (the folder containing `platformio.ini`).
2. Connect the ESP32 over USB. Close calibration, serial monitors, and any
   other program using its port.
3. Run **PlatformIO: Upload** from the command palette. The first build
   downloads the ESP32 toolchain and sensor libraries.

The root configuration builds `mlx90393_live.ino` for an ESP32 Dev Module.
PlatformIO detects the upload port automatically. If multiple boards are
connected, use its terminal and specify the desired port, for example on macOS:

```bash
pio run -t upload --upload-port /dev/cu.usbserial-0001
```

Use your own board's port; it can differ between laptops.

**Pulling updates does not change firmware already on a board.** After pulling
this channel update, upload once to each board that still uses channels
0, 2, 5, and 7. A board already updated can be used on another laptop without
uploading again.

To check readings, open a serial monitor at **115200 baud**. Every `FRAME`
should contain 12 numeric sensor values with no `nan`. Close the monitor
before running calibration.
