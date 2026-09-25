# Avocado Smash: Magnetic Edition

A lightweight local Pygame game for the eFlesh gesture classifier. It has no
web server, downloaded art, or online account: all graphics are drawn at
runtime and the keyboard works without sensor hardware.

## Run

From the repository root:

```bash
python3 -m pip install -r avocado_smash/requirements.txt
python3 avocado_smash/main.py
```

After calibration creates `calibration_pipeline/profile.json`, add the ESP32
serial port to play with live gestures:

```bash
python3 avocado_smash/main.py --sensor-port /dev/cu.usbserial-XXXX
```

On macOS, double-click `avocado_smash/Play.command`. The first launch creates
its local virtual environment and installs dependencies; later launches work
without reinstalling. From Terminal it also accepts sensor arguments:

```bash
./avocado_smash/Play.command --sensor-port /dev/cu.usbserial-XXXX
```

## Play

Avocados arc through the highlighted **ACT HERE** zone. Perform the matching
action while they are inside it:

| Target | Gesture | Keyboard |
| --- | --- | --- |
| Fresh avocado | Spread to slice | J |
| Armored avocado | Fist to smash | K |
| Rotten avocado | Wrist Up to deflect | F |

Fresh avocados can appear in groups. A correct target scores 10 points per
target; every five successful opportunities raises the multiplier to a maximum
of ×4. Missing fresh or armored avocados breaks the combo. Missing a rotten
avocado costs 10 points, and a wrong action inside the zone costs 5 points.

Each round lasts 60 seconds. Space/Enter starts or restarts, P pauses, and Esc
quits. Pausing resumes with a three-second countdown so queued gestures do not
affect the round.

The sensor panel shows Rest, Spread, Fist, and Wrist Up as relative KNN
proximity scores. They are useful live likelihood-style indicators but are not
calibrated probabilities. Sensor actions use the existing stable gesture-onset
detector; keyboard actions remain available in sensor mode.
