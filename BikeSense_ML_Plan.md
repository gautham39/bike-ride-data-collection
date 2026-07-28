# BikePulse — ML Implementation Guide for Motorcycle Predictive Maintenance

A detailed, build-ready plan for using 6-axis IMU data (3-axis accelerometer + 3-axis gyroscope) to detect riding patterns and predict component wear from vibration signatures.

---

## 0. The Core Constraint: Labels

The biggest decision driver is **whether you have failure labels**.

- Only data from a **normally-running bike** → **unsupervised anomaly detection** (most likely your case).
- Examples of **failing components** (worn sprocket, loose bearing, wheel imbalance) → add **supervised fault classification**.

Recommended path: start unsupervised, add supervised labels opportunistically as failures occur or are induced.

---

## 1. Data Acquisition Parameters

Get this layer right or nothing downstream works.

### Sensor & Mounting
| Parameter | Recommended Value | Reasoning |
|-----------|-------------------|-----------|
| IMU | MPU-9250 / ICM-42688 / BMI270 (or LSM6DSOX) | Low-noise MEMS 6-axis; ICM-42688 has low vibration noise density |
| Mount location | Rigid: engine case, swingarm, or frame near suspect component | Soft mounts (seat, bar) filter out the vibration you want |
| Mount coupling | Bolt/hard-epoxy, NOT foam or tape | Compliant mounts act as a low-pass filter and destroy fault signatures |
| Orientation | Fixed & recorded (know which axis is vertical/lateral/longitudinal) | Needed for interpretability |

### Sampling
| Parameter | Recommended Value | Reasoning |
|-----------|-------------------|-----------|
| Sampling rate (fs) | **1000–3333 Hz** for vibration; min 500 Hz | Nyquist: bearing/gear faults produce energy up to several hundred Hz+. Engine firing + harmonics need headroom. |
| Accel range | ±16 g (start), verify no clipping | Engine/road vibration can spike; clipping is silent data loss |
| Gyro range | ±2000 dps | Cornering & steering transients |
| Anti-alias filter | Enable IMU internal LPF at ~fs/2.5 | Prevents aliasing high-freq vibration into your band |
| Resolution | 16-bit | Standard MEMS |

> **Rule of thumb:** set `fs >= 2.56 × highest_fault_frequency_you_care_about`. If you want clean spectra up to 500 Hz, sample ≥ 1280 Hz; 2000 Hz is a safe default.

### Metadata to log alongside IMU (critical for context)
- Wheel speed / GPS speed
- Engine RPM (from ECU/OBD if available) — **hugely valuable**, ties vibration to a rotating reference
- Throttle position, gear
- Ambient temperature
- Timestamp (synchronized, monotonic)

---

## 2. Preprocessing Pipeline

```
Raw 6-axis @ fs
  → 1. Sync & de-dupe timestamps (resample to fixed fs)
  → 2. Gravity removal / orientation compensation (sensor fusion)
  → 3. Detrend + high-pass filter (remove DC & slow drift)
  → 4. Segment into windows
  → 5. Feature extraction (time + frequency domain)
  → 6. Normalize / scale
  → 7. Riding-context tagging
```

### 2.1 Sensor Fusion (gravity removal)
- Use a **Madgwick** or **complementary filter** to estimate orientation from accel+gyro, then subtract the gravity vector so linear vibration is isolated from tilt.
- Complementary filter coefficient `alpha`: **0.98** (trust gyro short-term, accel long-term). Tune 0.95–0.99.
- Madgwick `beta`: **0.033–0.1** (higher = trusts accel more, faster convergence, noisier).

### 2.2 Filtering
| Filter | Parameter | Value |
|--------|-----------|-------|
| High-pass (remove DC/drift) | cutoff | 0.5–2 Hz, 2nd–4th order Butterworth |
| Optional band-pass for bearing zone | band | e.g. 500–2000 Hz, depends on component |

### 2.3 Windowing
| Parameter | Recommended Value | Notes |
|-----------|-------------------|-------|
| Window length | **1–2 s** (e.g. 2048 samples @ ~1024 Hz) | Use a power of 2 for FFT efficiency |
| Overlap | **50%** | More training windows, smoother detection |
| Windowing function | **Hann** (for FFT) | Reduces spectral leakage |

### 2.4 Scaling
- Fit `StandardScaler` (zero mean, unit variance) **on healthy training data only**, then apply to all. Never fit on test/anomalous data — that leaks information.

---

## 3. Feature Extraction

Compute **per axis** (6 axes) → concatenate into one feature vector per window.

### Time-domain features (per axis)
| Feature | Formula / Meaning | Sensitive to |
|---------|-------------------|--------------|
| Mean | average | DC offset / mounting shift |
| RMS | sqrt(mean(x²)) | Overall energy / severity |
| Std / Variance | spread | General instability |
| Peak / Peak-to-peak | max−min | Impacts, looseness |
| **Kurtosis** | 4th moment | **Early bearing faults (impulsiveness)** |
| Skewness | 3rd moment | Asymmetric faults |
| **Crest factor** | peak / RMS | **Impulsive faults before RMS rises** |
| Shape factor | RMS / mean(|x|) | Waveform change |
| Zero-crossing rate | — | Frequency shift proxy |

### Frequency-domain features (per axis, via FFT)
| Feature | Meaning |
|---------|---------|
| Spectral energy in bands | Split 0–fs/2 into bands (e.g. 6–10 bands), sum power each |
| Dominant frequency + amplitude | Peak of spectrum |
| Spectral centroid | "Center of mass" of spectrum |
| Spectral entropy | Flatness/complexity of spectrum |
| PSD (Welch) | Robust power spectral density estimate |
| Envelope-spectrum peaks | **Best for bearing defect frequencies** (demodulate then FFT) |

> **If you have RPM:** compute **order-domain** features (resample vs shaft angle). Fault frequencies then stay at fixed "orders" regardless of speed — dramatically more robust for a variable-speed machine like a bike.

**Feature count:** ~15 time-domain + ~15 freq-domain per axis × 6 axes ≈ **~180 features/window**. Reduce later with PCA if needed.

---

## 4. Riding-Context Separation (do NOT skip)

A pothole ≠ a failing bearing. Motorcycle vibration is dominated by riding mode.

**Strategy:** Only run fault detection within a **known steady state**, or feed context as an input.

1. **Mode classifier** (lightweight): classify each window into {idle, steady-cruise, accel, braking, cornering, rough-road} using speed/RPM/gyro.
2. **Gate detection:** Run the anomaly model **only on `idle` and `steady-cruise` windows** where the baseline is stable and repeatable.
3. Alternatively, **condition on RPM**: build separate baselines per RPM bin (e.g. 1500–2500, 2500–3500 …), because vibration signature changes with speed.

This single step prevents the majority of false alarms.

---

## 5. Algorithm Implementation

### TIER 1 — Isolation Forest (START HERE)

**When:** No labels, want fast interpretable baseline, edge-friendly.

**Parameters:**
| Param | Value | Notes |
|-------|-------|-------|
| `n_estimators` | 100–200 | More trees = stabler score |
| `max_samples` | 256 (or 'auto') | Subsample per tree |
| `contamination` | 'auto' or set from expected anomaly rate (e.g. 0.01) | Controls threshold; better to threshold manually (see §6) |
| `max_features` | 1.0 | Use all features |
| `random_state` | fixed (e.g. 42) | Reproducibility |

```python
from sklearn.ensemble import IsolationForest
from sklearn.preprocessing import StandardScaler
import numpy as np

# X_train = healthy windows only, shape (n_windows, n_features)
scaler = StandardScaler().fit(X_train)
Xtr = scaler.transform(X_train)

iso = IsolationForest(
    n_estimators=150, max_samples=256,
    contamination='auto', max_features=1.0, random_state=42
)
iso.fit(Xtr)

# Higher score = more normal; we use negative for "anomaly score"
def anomaly_score(X):
    return -iso.score_samples(scaler.transform(X))

# Set threshold from healthy validation set (e.g. 99th percentile)
val_scores = anomaly_score(X_val_healthy)
threshold = np.percentile(val_scores, 99)

# Flag
scores = anomaly_score(X_test)
flags = scores > threshold
```

**Alternative Tier 1:** One-Class SVM (`kernel='rbf'`, `nu=0.01–0.05`, `gamma='scale'`). Slower on large data; Isolation Forest usually preferred.

**Supervised Tier 1 (if labels exist):** Random Forest / XGBoost.
- XGBoost: `n_estimators=300`, `max_depth=6`, `learning_rate=0.05`, `subsample=0.8`, `colsample_bytree=0.8`, early stopping on val loss. Handle class imbalance with `scale_pos_weight` or class weights.

---

### TIER 2 — LSTM Autoencoder

**When:** More data available, want temporal modeling + "which axis" diagnostics.

Train **only on normal data**; high reconstruction error ⇒ anomaly.

**Two input options:**
- **A) Feature sequences:** sequence of the ~180-dim feature vectors (recommended, lighter).
- **B) Raw windows:** downsampled raw signal (heavier, needs conv front-end → see Tier 3).

**Architecture (feature-sequence version):**
```
Input: (timesteps=T, features=F)
Encoder:  LSTM(128) -> LSTM(64)  (return last state)  -> latent (64)
RepeatVector(T)
Decoder:  LSTM(64) -> LSTM(128) -> TimeDistributed(Dense(F))
Output: reconstructed (T, F)
```

**Hyperparameters:**
| Param | Value |
|-------|-------|
| Sequence length T | 10–30 windows (e.g. 20 windows × 1 s = 20 s context) |
| Latent dim | 32–64 |
| Hidden units | 128 → 64 (encoder), mirrored decoder |
| Dropout | 0.2 |
| Loss | MSE (reconstruction) |
| Optimizer | Adam, lr = 1e-3 (reduce-on-plateau to 1e-4) |
| Batch size | 32–64 |
| Epochs | 50–200 with **early stopping** (patience 10, monitor val_loss) |
| Train/val split | 80/20 of **healthy** data only |

```python
import tensorflow as tf
from tensorflow.keras import layers, Model

T, F = 20, X_train.shape[1]

inp = layers.Input(shape=(T, F))
e = layers.LSTM(128, return_sequences=True)(inp)
e = layers.LSTM(64, return_sequences=False)(e)
z = layers.RepeatVector(T)(e)
d = layers.LSTM(64, return_sequences=True)(z)
d = layers.LSTM(128, return_sequences=True)(d)
out = layers.TimeDistributed(layers.Dense(F))(d)

ae = Model(inp, out)
ae.compile(optimizer=tf.keras.optimizers.Adam(1e-3), loss='mse')

es = tf.keras.callbacks.EarlyStopping(patience=10, restore_best_weights=True)
rlr = tf.keras.callbacks.ReduceLROnPlateau(patience=5, factor=0.5)
ae.fit(S_train, S_train, validation_data=(S_val, S_val),
       epochs=200, batch_size=64, callbacks=[es, rlr])

# Reconstruction error per sequence
def recon_error(S):
    pred = ae.predict(S)
    return np.mean((S - pred)**2, axis=(1, 2))

val_err = recon_error(S_val)                 # healthy
threshold = np.mean(val_err) + 3*np.std(val_err)   # or 99th percentile
anomaly = recon_error(S_test) > threshold

# Per-feature error -> which axis is degrading
per_feat = np.mean((S_test - ae.predict(S_test))**2, axis=1)  # (n, F)
```

**Threshold setting:** `mean + 3σ` of healthy reconstruction error, OR 99th percentile. Validate on a held-out healthy set to confirm the false-alarm rate matches expectation.

---

### TIER 3 — Hybrid CNN-LSTM (best accuracy)

**When:** Max accuracy, off-board compute, ideally some labeled faults.

- **CNN front-end** extracts local vibration signatures from raw/near-raw windows.
- **LSTM** models temporal degradation across windows.
- Optionally add **attention** and a small **MLP classifier** head for fault typing.

**Architecture sketch:**
```
Raw window (samples, 6) 
 -> Conv1D(32, k=7) -> BN -> ReLU -> MaxPool
 -> Conv1D(64, k=5) -> BN -> ReLU -> MaxPool
 -> Conv1D(128,k=3) -> BN -> ReLU -> GlobalPool  (per-window embedding)
 -> stack embeddings over N windows
 -> LSTM(128) -> LSTM(64)
 -> [Detection] Dense reconstruction  (unsupervised)  OR
 -> [Classification] Dense(softmax over fault classes)  (supervised)
```

**Hyperparameters:**
| Param | Value |
|-------|-------|
| Conv filters | 32 → 64 → 128 |
| Kernel sizes | 7, 5, 3 |
| LSTM units | 128 → 64 |
| Dropout | 0.2–0.3 |
| Optimizer | Adam 1e-3, cosine/plateau decay |
| Loss | MSE (detection) or categorical cross-entropy (classification) |
| Batch | 32 |
| Regularization | BatchNorm + dropout + early stopping |

**Recommended combined design:** unsupervised CNN-LSTM-AE for *detection* + small supervised classifier for *fault typing* once labels accrue.

---

## 6. Thresholding & Alerting

- **Score threshold:** derive from a **held-out healthy set**, not from anomalies. Use 99th percentile or mean+3σ.
- **Persistence filter:** require **N consecutive** anomalous windows (e.g. 5–10) before alerting → kills transient false alarms from bumps.
- **Trend tracking:** track a rolling mean of anomaly score over rides. Predictive maintenance value comes from the **upward trend over days/weeks**, not a single spike.
- **Severity bands:** e.g. green < T1, amber T1–T2, red > T2.

---

## 7. Test Conditions

Design data collection to cover the real operating envelope AND induce faults safely.

### 7.1 Baseline (healthy) collection — must span the full envelope
| Variable | Conditions to capture |
|----------|-----------------------|
| Speed | idle, 20, 40, 60, 80, 100+ km/h |
| RPM | across the usable rev range, per gear |
| Gear | each gear |
| Road surface | smooth tarmac, coarse tarmac, gravel, cobbles |
| Maneuver | steady cruise, accel, braking, cornering |
| Load | solo vs pillion (if relevant) |
| Temperature | cold start vs warmed up |
| Repeats | ≥ 5 rides per condition on different days for variance |

> Aim for **many hours** of healthy data across days/weeks. Baseline richness determines false-alarm rate.

### 7.2 Fault conditions (to validate detection — induce safely & legally)
Only where safe and reversible. Examples:
| Fault | How to simulate | Expected signature |
|-------|-----------------|--------------------|
| Wheel imbalance | Add small known weight to rim | 1× wheel-rotation-frequency peak grows |
| Chain/sprocket wear | Loosen chain to spec-min tension | Increased low-freq impulsiveness / kurtosis |
| Loose fastener | Reduce a non-critical bolt torque | Broadband + rattle harmonics |
| Bearing wear | Test rig with worn bearing (bench, not road) | Envelope-spectrum defect-frequency peaks |
| Tire pressure | Under-inflate 20–30% | Shift in low-freq vibration & damping |

**Safety first:** never induce faults that compromise braking, steering, or structural integrity on-road. Use a bench/dyno/test rig for anything risky.

### 7.3 Data splits
- **Train:** healthy only (unsupervised) — 70%.
- **Validation:** healthy only — 15% (for thresholds & early stopping).
- **Test:** held-out healthy 15% **+ all fault data** (only used at the very end).
- **Split by ride/day, not by window** — windows from the same ride are correlated; random window splits leak and inflate scores.

---

## 8. Validation & Metrics

### 8.1 Anomaly detection metrics (primary)
Since you likely have imbalanced/limited fault data:
| Metric | Why |
|--------|-----|
| **Precision / Recall / F1** | Core — recall = caught faults, precision = few false alarms |
| **ROC-AUC** | Threshold-independent ranking quality |
| **PR-AUC (Average Precision)** | **Better than ROC-AUC under class imbalance** |
| **False Alarm Rate** (on healthy test) | Directly = nuisance alerts |
| **Detection lead time** | How early before failure it flags — the real PdM value |
| **Confusion matrix** | Where it fails |

### 8.2 Classification metrics (if supervised fault typing)
- Per-class precision/recall/F1, macro-F1, confusion matrix.
- Report accuracy **with** macro-F1 (accuracy alone hides minority-class failure).

### 8.3 Cross-validation strategy
- **Group / leave-one-ride-out CV** (group = ride or day) — prevents leakage from correlated windows.
- For time series, prefer **rolling/expanding-window CV** over shuffled k-fold.
- Report **mean ± std** across folds to show stability, not a single lucky run.

### 8.4 Robustness checks (don't skip)
- **False-alarm test:** run on fresh *healthy* rides not seen in training. Target a defined FAR (e.g. < 1 alert / hour).
- **Sensitivity to mounting:** re-mount the sensor, confirm the model still baselines correctly (or requires re-calibration).
- **Speed/RPM generalization:** confirm no false alarms when hitting speeds under-represented in training.
- **Temperature drift:** verify cold-start rides don't false-trigger.
- **Ablation:** compare feature-only vs raw-input, with vs without RPM/order-tracking, to justify complexity.

### 8.5 Success criteria (example targets — tune to your risk tolerance)
| Metric | Target |
|--------|--------|
| Recall on induced faults | ≥ 0.90 |
| Precision | ≥ 0.80 |
| PR-AUC | ≥ 0.85 |
| False alarm rate (healthy) | ≤ 1 / hour of steady-state riding |
| Detection lead time | Flags before functional failure in trend |

---

## 9. Deployment Notes

| Concern | Guidance |
|---------|----------|
| Edge (ESP32 / MCU) | Tier 1 (Isolation Forest / feature + tree) or a tiny quantized model; do FFT/feature extraction on-device |
| Phone / Raspberry Pi | Tier 2 feasible (TFLite) |
| Off-board / cloud | Tier 3 fine |
| Model format | TFLite / ONNX for edge; quantize to int8 where possible |
| Drift over time | Schedule periodic **re-baselining**; components wear normally too, so the "healthy" model must adapt or be retrained on recent clean data |
| Logging | Keep raw + features for a rolling window so flagged events can be inspected & labeled retroactively |

---

## 10. Recommended Build Order

1. Nail **acquisition** (mounting, fs, ranges, no clipping) + log RPM/speed.
2. Build **preprocessing + feature extraction** pipeline.
3. Add **riding-context gating** (steady-state only).
4. Ship **Isolation Forest baseline** → set threshold from healthy data → measure FAR.
5. Collect induced-fault data → validate with §8 metrics.
6. Upgrade to **LSTM Autoencoder** as data grows.
7. Consider **CNN-LSTM** only if accuracy demands it and compute allows.

---

## Quick Reference

| Tier | Method | Labels? | Edge? | Key Params |
|------|--------|---------|-------|------------|
| 1 | Isolation Forest | No | Yes | n_estimators=150, max_samples=256, threshold=99th pct |
| 1 | One-Class SVM | No | Yes | nu=0.01–0.05, rbf, gamma='scale' |
| 1 | XGBoost | Yes | Yes | depth=6, lr=0.05, n=300, early stop |
| 2 | LSTM Autoencoder | No | Harder | T=20, latent=32–64, mean+3σ threshold |
| 3 | CNN-LSTM (+MLP) | Partial | Hardest | conv 32/64/128, LSTM 128/64, dropout 0.2–0.3 |

---

## Open Questions to Finalize

1. Any **labeled failure data**, or only healthy-bike data?
2. Inference target: **on-device (ESP32/edge)** or **off-board (phone/laptop)**?
3. Can you read **engine RPM** (OBD/ECU)? If yes, use order-domain analysis — big robustness win.
