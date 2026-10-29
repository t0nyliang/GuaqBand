---
theme: default
title: "Preprocessing"
info: GuaqBand preprocessing, features, and gesture decisions
colorSchema: light
aspectRatio: 16/9
canvasWidth: 1100
fonts:
  sans: Arial
  mono: Menlo
  provider: none
  local: Arial, Menlo
highlighter: shiki
transition: none
drawings:
  persist: false
class: cover
---
# Preprocessing

**Filtering, baseline-relative features, and gesture decisions.**

Begin with the GuaqBand preprocessing pipeline.

---
class: section
---
# 1 · Building GuaqBand

**Build the preprocessing stages of the gesture detector.**

Open `preprocessing/preprocessing_lab.ipynb`.

Implement smoothing, features, distances, and a nearest-neighbor vote.

<!--
The introductory review is removed. Lecture sections are numbered 1–3 and labs 1–4; notebook sections are referenced explicitly below.
-->

---

# What are we building?

Four MLX90393 sensors measure three magnetic-field axes each.

- **12 channels:** s0x, s0y, s0z, through s3z, in µT
- **50 Hz target:** about 20 ms between frames
- **Four labels:** rest, wrist_up, spread, fist


<!--
Mux channels 0, 2, 5, 7 determine sensor order. Reads are sequential and grouped into near-synchronous frames. The separate BNO085 MOTION stream is outside this lesson.
Source: README.md and mlx90393_live/mlx90393_live.ino in this repository.
-->

---

# One frame and one processing history

```text
FRAME,sequence,device_us,s0x,s0y,s0z,...,s3z
```

The sequence tracks order. Device time tells us when frames arrived at the sensor stream.

The detector forms a **19 × 12 history** on a uniform 50 Hz grid before computing a feature vector.

**Reminder**: index ≠ time

<!--
Source: calibration_pipeline/eflesh_calibration/sensor.py and app.py. Firmware fields s1x through s2z are abbreviated here. Live history comes from device time. Guided calibration currently uses host elapsed acquisition time.
-->

---

# Our pipeline

| Stage | Output |
|---|---|
| Timestamp checks and resampling | 19 rows × 12 channels |
| Five-sample causal average | 15 rows × 12 channels |
| Relaxed baseline subtraction | Signed field changes |
| Mean and RMS per channel | 24-value feature vector |
| Training scaling and 3-neighbor vote | Raw gesture label |
| Two matching predictions | Published gesture label |

<!--
Source: calibration_pipeline/eflesh_calibration/knn.py and app.py. The table describes the default feature and live label path. Rest baseline collection precedes these rolling operations.
-->

---

# Remember the effectiveness of a low pass filter

<img class="figure" src="/figures/pulse-step.svg" />

Sudden changes become wider.

For us, it helps identify when a gesture actually starts.

<!--
The existing diagram is a mathematical three-tap example, not a real wearer recording. The current notebook shows student smoothing outputs on real training data or an explicitly labeled illustration.
-->

---

# Causal averaging

$$
y_c[n]=\frac{1}{5}\sum_{k=0}^{4}x_c[n-k]
$$

Average over 5 frames.

**What does causal mean?**

<!--
Source: calibration_pipeline/eflesh_calibration/knn.py, moving_average. Complete-window alignment corresponds to t[4:]. No zero padding or future samples enter the feature path.
-->

---
class: lab
---
# Lab 1 · A causal sensor filter

- Predict the output for a short pulse.
- Implement `moving_average` in notebook **§1**.
- Inspect the 19-input / 15-output plot and causal alignment.

**Return with:** an aligned trace and an explanation of noise reduction versus transition spreading.

---
class: section
---
# 2 · Features for the armband

**What information should we retain?**

The goal here is effective image compression..

---

# Relaxed field defines the reference

$$
b_c=\operatorname{mean}(\text{known relaxed samples for channel }c)
$$

Subtract the baseline after smoothing. Each channel has its own reference.

The live program collects two seconds of relaxed data. Recalibrate when wearer, fit, or sensor placement changes.

A baseline containing a held gesture erases part of that gesture's displacement.

<!--
Source: calibration_pipeline/eflesh_calibration/app.py collect_baseline, and knn.py feature. The notebook uses the known rest interval recorded in each trial manifest. Its illustration is not wearer evidence.
-->

---

# Mean preserves direction and RMS preserves size

$$
m_c=\operatorname{mean}(y_c-b_c),\qquad
r_c=\sqrt{\operatorname{mean}((y_c-b_c)^2)}
$$

| Constant field change | Signed mean | RMS |
|---|---|---|
| +3 µT | +3 µT | 3 µT |
| −3 µT | −3 µT | 3 µT |

Save **12 means followed by 12 RMS values** in sensor-axis order.

<!--
Source: calibration_pipeline/eflesh_calibration/knn.py feature and feature_layout. RMS is relative to the relaxed baseline, not variation around the window's own mean.
-->

---

# Features discard waveform detail

| Filtered field changes in one channel | Mean | RMS |
|---|---|---|
| +1, −1, 0 µT | 0 µT | $\sqrt{2/3}$ µT |
| −1, +1, 0 µT | 0 µT | $\sqrt{2/3}$ µT |

The order changed. The summaries stayed equal.

Signed mean and RMS suit a simple held-pose detector. They cannot preserve every movement's timing or order.

<!--
Three filtered rows illustrate the information loss. Production features summarize 15 filtered rows. Permuting those rows also preserves per-channel means and RMS. The notebook asks for examples after filtering, since raw reordering can change filtered moments.
-->

---
class: lab
---
# Lab 2 · The production feature contract

Implement `feature` in notebook **§2** and compare its outputs with production checks.

- Compare opposite constant changes.
- Contaminate the baseline with a held pose.
- Explain what the window summaries discard.

**Return with:** a feature plot, exact array shapes, and one feature-loss example.

---
class: section
---
# 3 · Gesture decisions

**How do we turn features into a useful live label?**

Notebook **§3:** implement KNN distances and voting, then inspect the selected neighbors.

---

# Why scale before finding neighbors?

KNN combines differences across **all 24 features**. Some features naturally vary much more than others.

| Feature | Difference between two examples | Training standard deviation | Scaled difference |
|---|---|---|---|
| A | 20 µT | 20 µT | 1 |
| B | 1 µT | 1 µT | 1 |

Without scaling, A contributes **400** to squared distance; B contributes **1**.

After scaling, both differences are one standard deviation, so each contributes **1**.

**Compare differences relative to each feature's usual variation.**

---

# A scaled value says how far from usual

$$
\text{scaled value}=\frac{\text{value} - \text{training mean}}{\text{training standard deviation}}
$$

| Feature | Value | Training mean | Training standard deviation | Scaled value |
|---|---|---|---|---|
| A | 120 | 100 | 20 | 1 |
| B | 3 | 2 | 1 | 1 |

Both values are **one standard deviation above their training average**.

Zero means the training average. Negative values are below it. KNN compares examples on these scaled axes.

<!--
Centering gives a useful interpretation. Subtracting the same mean from both points cancels in their difference; division by per-feature standard deviation changes their relative distance contributions. Standardization does not guarantee equal predictive importance.
-->

---

# Use the same ruler for every prediction

Compute and save the 24 means and standard deviations from **training examples only**.

Use those same numbers for training, tuning, test, and live inputs.

| Operation | Question it answers |
|---|---|
| Relaxed-baseline subtraction | How much did the field change from a relaxed hand? |
| Feature standardization | How large is this feature relative to its training variation? |


<!--
Source: calibration_pipeline/eflesh_calibration/knn.py make_model and normalized_distances. Near-zero means <=1e-9. Runtime calibration uses all collected examples as training; the notebook adds whole-trial tuning/test splits.
-->

---

# KNN stores labeled examples

Each training row is **24 features + one gesture label**.

A new query is another 24-value vector. Find examples with similar feature values.

**K = 3:** let the three closest examples vote.

---

# How far is one example from the query?

For standardized training example $z_i$ and standardized query $q$:

$$
d_i=\sqrt{\sum_{j=0}^{23}(z_{ij}-q_j)^2}
$$

Measure the difference in each of the 24 features, then combine them into one distance.

---

# See the three nearest neighbors

<img class="figure" src="/figures/knn-neighbors.svg" />

This illustration uses two features. Your implementation measures distance across **all 24**.

---

# Three neighbors vote for a gesture

| Neighbor, closest first | Distance | Label |
|---|---|---|
| 1 | 0.3 | fist |
| 2 | 0.4 | rest |
| 3 | 0.6 | fist |

**Prediction: fist**, with two of three votes. The closest example alone does not decide.

If all three labels differ, choose the nearest label.


---
class: lab
---
# Lab 3 · Implement KNN

Notebook **§3:** implement `normalized_distances(model, sample)`.

In the same section, implement `predict(model, sample)`, calling your distance function.

- Use saved training scales and `model['k']`.
- Pass distance, majority-vote, and tie checks.
- Inspect your neighbors, distance bars, and vote plot.

**Return with:** your predicted label and the neighbors supporting it.
