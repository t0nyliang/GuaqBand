# Week 3: Make the ESP32 talk to the magnetometers

**Goal:** the ESP32's blue LED lights up when you press the pad. Then you use Cursor to extend it.
**Time:** ~30 min (do Setup beforehand).

| Step | What | Time |
|---|---|---|
| 0 | How I2C works (read) | 3 min |
| 1 | Find the sensor | 5 min |
| 2 | LED on press | 10 min |
| 3 | Extend with Cursor | 10 min |
| 4 | Demo | 2 min |

You only fill in blanks marked `___`. Each one has a **Hint** and a **Format** comment above it.

---

## Setup (before the session)

1. In Cursor: **Extensions** (`Ctrl+Shift+X`), then install **PlatformIO IDE**. Restart when asked.
2. **File > Open Folder** and pick `notebooks/week3`. It must be this folder, not the repo root.
3. Plug in the ESP32 with a USB **data** cable.
4. Click **✓ Build** once. The first build downloads everything (a few minutes).
   The red underlines on `#include` lines go away after this first build.

**PlatformIO buttons** (bottom status bar):

| Button | Does |
|---|---|
| ✓ | Build |
| → | Upload to the ESP32 |
| 🔌 | Serial monitor (see what the ESP32 prints) |

**Switch sketches:** in `platformio.ini`, change `src_dir = week3_led` to the folder you want, then save.

**If something goes wrong:**
- **No port / upload can't find the board:** try another cable, or install the
  [CP210x](https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers) or
  [CH340](https://www.wch-ic.com/downloads/CH341SER_EXE.html) driver. The chip name is printed near the USB socket.
- **Stuck on `Connecting....`:** hold the **BOOT** button until the upload starts.
- **`Port is busy`:** close the serial monitor before you upload.
- **Monitor prints nothing:** press **EN** on the board.

---

## Step 0: How it works (3 min)

**I2C** = two wires (SDA, SCL). Every chip has an **address**, and the ESP32 talks to one address at a time.

The full armband has four MLX90393 magnetometers, all with the **same** address (`0x18`).
So they sit behind a **PCA9548 mux** (`0x70`), a switch that connects one channel at a time.
Today your mux has just **one** magnetometer plugged in:

```text
ESP32 ── PCA9548 mux (0x70) ── ch ? ── MLX90393 (0x18)
```

**To read the sensor:** (1) tell the mux which channel, (2) talk to `0x18`.

---

## Step 1: Find the sensor (5 min)

Set `src_dir = week3_scan`. Fill in **2 blanks** in `week3_scan/week3_scan.ino`, upload, and open the monitor.

✅ **You should see:** `channel N: MLX90393 at 0x18` and `Found 1 sensor(s)`. Remember **N**.
Nothing found? Check the cables.

---

## Step 2: LED on press (10 min)

Set `src_dir = week3_led`. In `week3_led/week3_led.ino`, set `SENSOR_CHANNEL` to your **N**, fill in
**4 blanks** in `loop()`, then upload.

- When it starts, it records what the sensor reads at rest. **Don't touch the pad for 1 second.**
- The monitor prints `change:`. It's near 0 at rest and jumps when you press.
- **Tune `THRESHOLD_UT`** (top of the file) to sit between "resting" and "light press", then re-upload.

✅ **Done when:** the LED is off at rest and turns on when you press over the sensor.

Stuck for more than 3 minutes? Look at `week3_answers/week3_answers.ino`.

---

## Step 3: Extend with Cursor (10 min)

The board has **one blue LED**, so instead of colors, use **patterns or brightness**. Pick one:

- **A. Brightness:** the harder the press, the brighter the LED.
- **B. Direction:** blink 1, 2 or 3 times depending on whether x, y or z changed most.
- **C. Tap vs. hold:** a tap gives one flash; holding for more than 1 s gives fast blinking.

**1. Save first.** Commit, or note the Cursor checkpoint, so you can undo.
Don't save a backup `.ino` in the same folder, because it breaks the build.

**2. Prompt.** Cursor doesn't know your hardware. Paste this, and fill in the goal:

```text
Goal: <option A, B or C in your own words>

Hardware (don't change):
- ESP32 DevKit, Arduino framework in PlatformIO (Arduino-ESP32 core 2.x)
- I2C SDA=21, SCL=22, PCA9548 mux at 0x70
- 1 MLX90393 at 0x18 on mux channel <N>
- Only output: one blue LED on GPIO2. No RGB LED, no NeoPixel.

Files: @week3_led.ino (my working code)

Rules: edit only week3_led.ino, no new libraries, no long delay() in loop().

First give me a short plan. Don't write code until I say go.
```

**Tips:**
- Telling it **what's not there** ("no RGB LED") avoids the most common wrong answer.
- If it doesn't compile, paste the **exact error** back.
- If the behavior is wrong, describe **what you saw** ("LED stays on after I let go").

**3. Check before uploading:**
- [ ] Pins and addresses unchanged (21/22, `0x70`, `0x18`, your channel, GPIO2)?
- [ ] Can you explain every changed line? If not, ask Cursor to explain it, or undo it.

---

### Facilitator notes
- Before the session, run `week3_answers` on one kit and set a real default `THRESHOLD_UT` (100 is a placeholder).
- Step 1 answers are in the header of `week3_answers.ino`.
