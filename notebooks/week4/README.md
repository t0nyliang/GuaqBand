# Week 4: Four sensors, four colors

**Goal:** press any of the four pads and an RGB LED shows that pad's color. Press two or more at once and it flashes white. Then you use Cursor to extend it.
**Time:** ~40 min (do Setup beforehand).

| Step | What | Time |
|---|---|---|
| 0 | How the mux reads four sensors + why baselines drift (read) | 4 min |
| 1 | Four colors + drift fix (`week4_colors`) | 15 min |
| 2 | Two or more pressed = flashing white (`week4_multi`) | 8 min |
| 3 | Extend with Cursor | 10 min |
| 4 | Demo | 3 min |

You only fill in blanks marked `___`. Each one has a **Hint** and usually a **Format** comment above it.

---

## Setup (before the session)

Same as Week 3, plus one NeoPixel:

1. **File > Open Folder** and pick `notebooks/week4`. It must be this folder, not the repo root or `week3`.
2. Wire the **NeoPixel** (WS2812 RGB LED):

   | NeoPixel | ESP32 |
   |---|---|
   | DIN (data in) | GPIO4 |
   | 5V / VCC | VIN (5V) or 3V3 |
   | GND | GND |

3. Check all **four** magnetometers are plugged into the mux on channels **0, 2, 5, 7**.
   (Different channels? Run Week 3's `week3_scan` and change `CHANNELS` at the top of the sketch.)
4. Plug in the ESP32 with a USB **data** cable and click **✓ Build** once. The first build downloads the NeoPixel library.

**PlatformIO buttons** (bottom status bar): ✓ Build · → Upload · 🔌 Serial monitor.

**Switch sketches:** in `platformio.ini`, change `src_dir = week4_colors` to the folder you want, then save.

**If something goes wrong:**
- **Upload can't find the board / stuck on `Connecting....` / `Port is busy`:** see the Week 3 README. Usually the fix is to hold **BOOT** while it connects, or to close the serial monitor.
- **`Sensor N (channel C) not found`:** that sensor's cable is loose or it's on a different channel. The other sensors keep working.
- **NeoPixel stays dark:** check DIN goes to GPIO4 (not DOUT) and GND is shared. Hold a pad while testing, because the LED is off at rest.

---

## Step 0: How it works (4 min)

### Reading four sensors through one mux

All four MLX90393s have the **same** address (`0x18`). If they shared one I2C bus they'd all answer at once.
The **PCA9548 mux** (`0x70`) fixes that: it connects exactly one channel to the ESP32 at a time.

```text
                    ┌─ ch 0 ── sensor 0 (0x18)  red
ESP32 ── mux 0x70 ──┼─ ch 2 ── sensor 1 (0x18)  green
                    ├─ ch 5 ── sensor 2 (0x18)  blue
                    └─ ch 7 ── sensor 3 (0x18)  yellow
```

So every loop goes **one sensor at a time**:

```text
for each sensor i:
    1. switch the mux to CHANNELS[i]   ← now only sensor i is connected
    2. read x, y, z from 0x18          ← this is sensor i's data
```

The reads are back to back (a few ms each), so it *looks* simultaneous, but they're actually one after another.
Notice the sensor **number** (0-3) and the mux **channel** (0, 2, 5, 7) are different things. The `CHANNELS` array translates between them.

### Why the baseline drifts

Like Week 3, each sensor records a **baseline** at startup (its value at rest), and "pressed" means "far from the baseline".
But the resting value slowly **wanders**:
- the armband shifts on your arm,
- the temperature changes,
- you turn your arm, and the sensors see Earth's magnetic field from a new angle.

After a while, an untouched pad can look "pressed". The fix: every **10 seconds**, take each sensor's current reading as its new baseline.
**But only for sensors nobody is pressing.** Otherwise the press becomes the new "normal".

---

## Step 1: Four colors + drift fix (15 min)

Set `src_dir = week4_colors`. In `week4_colors/week4_colors.ino`, fill in **6 blanks**:

| TODO | Where | What it teaches |
|---|---|---|
| 1 | `readSensor()` | point the mux at the right channel |
| 2 | `loop()` | loop over all four sensors |
| 3 | `loop()` | pressed / not pressed (same idea as Week 3) |
| 4 | `loop()` | pick that sensor's color |
| 5 | `loop()` | "has 10 s passed?" using `millis()` |
| 6 | `loop()` | only re-baseline sensors that aren't pressed |

Upload, open the monitor, and **don't touch the pads for 1 second** while it records baselines.
You'll see a line like `s0:3.1 s1:2.4 s2:5.0 s3:1.8 threshold:100.0`. Each `sN` should be near 0 at rest and jump when you press that pad.
**Tune `THRESHOLD_UT`** so it sits between "resting" and "light press", then re-upload.

✅ **Done when:**
- [ ] Each pad lights its own color: 0 red, 1 green, 2 blue, 3 yellow.
- [ ] **Drift test:** rotate your arm or the board slowly until an `sN` value creeps up, then wait. Within 10 s you see `--- re-baselined ---` and it drops back to near 0.
- [ ] **Hold test:** hold one pad through a `--- re-baselined ---` line. The LED should stay on. (If it turns off, check TODO 6.)

---

## Step 2: Two or more = flashing white (8 min)

Set `src_dir = week4_multi`. The file `week4_multi/week4_multi.ino` is the finished Step 1 code plus **4 new blanks** in `loop()`:

| TODO | What it teaches |
|---|---|
| 1 | count how many pads are pressed |
| 2 | "two or more" as a comparison |
| 3 | blink using the clock (`millis()` and `%`) instead of `delay()` |
| 4 | the `a ? b : c` shortcut |

Copy your tuned `THRESHOLD_UT` from Step 1 into this file, then upload.

✅ **Done when:** one pad shows its color, and two or more pads flash white. The monitor's `pressed:` count matches what you're pressing.

**Why not `delay()` for flashing?** While `delay(150)` waits, nothing else runs: no sensors are read and the LED can't react.
With `millis()` the loop keeps spinning, and the clock decides whether the LED is on or off.

Stuck for more than 3 minutes? Look at `week4_answers/week4_answers.ino`. Its header also has every Step 1 answer.

---

## Step 3: Extend with Cursor (10 min)

Pick one (or invent your own):

- **A. Count colors:** a different color for each number pressed, e.g. 2 = purple, 3 = cyan, 4 = rainbow cycle.
- **B. Mix colors:** blend the colors of the pressed pads, e.g. red + blue → purple, instead of white.
- **C. Brightness:** the harder you press (bigger `change`), the brighter the color.
- **D. Swipe:** press pad 0 then pad 1 within half a second → a special color or animation.
- **E. Manual reset:** hold all four pads for 2 s → re-record every baseline right now (flash green when done).

**1. Save first.** Commit, or note the Cursor checkpoint, so you can undo.
Don't save a backup `.ino` in the same folder, because it breaks the build.

**2. Prompt.** Cursor doesn't know your hardware. Paste this and fill in the goal:

```text
Goal: <option A-E, or your own idea, in your own words>

Hardware (don't change):
- ESP32 DevKit, Arduino framework in PlatformIO (Arduino-ESP32 core 2.x)
- I2C SDA=21, SCL=22, PCA9548 mux at 0x70
- 4 MLX90393 magnetometers, all at 0x18, on mux channels 0, 2, 5, 7
  (CHANNELS array). Select the channel before every read: readSensor(i) does this.
- Output: ONE NeoPixel (WS2812) on GPIO4, Adafruit_NeoPixel library,
  colors as 0xRRGGBB, set with showColor(color).

How the code works:
- loop() reads all 4 sensors, computes change[i] (uT from baseline) and pressed[i].
- Baselines re-record every REBASELINE_MS, only for sensors not pressed.

Files: @week4_multi.ino (my working code)

Rules: edit only week4_multi.ino, no new libraries, no delay() longer than
20 ms in loop() (use millis() for timing), keep the drift re-baseline working.

First give me a short plan. Don't write code until I say go.
```

**Tips:**
- If it doesn't compile, paste the **exact error** back.
- If the behavior is wrong, describe **what you saw**, e.g. "it flashes white when I only press pad 2".
- Ask it to **explain** any line you don't understand before uploading.

**3. Check before uploading:**
- [ ] Pins and addresses unchanged (21/22, `0x70`, `0x18`, channels 0/2/5/7, GPIO4)?
- [ ] Still no long `delay()` in `loop()`?
- [ ] Hold a pad through a `--- re-baselined ---` line. Does the LED stay on?

---

### Facilitator notes
- Before the session, run `week4_answers` on one kit and set a real default `THRESHOLD_UT` in both sketches (100 is a placeholder).
- `REBASELINE_MS` is 10 s so students can *see* the drift fix during class. On a real armband, 30-60 s (or a slow running average) is gentler.
- The re-baseline uses one reading, not an average. That's fine while the threshold is well above the resting noise.
- NeoPixel brightness is capped at 40/255 in `setup()` so it isn't blinding and draws little current over USB.
