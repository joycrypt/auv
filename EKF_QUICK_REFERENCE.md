# EKF Quick Reference - One Page Summary

## The Big Picture

```
       ┌─────────────────────────────────────────────┐
       │         Extended Kalman Filter Loop          │
       └─────────────────────────────────────────────┘
                           │
        ┌──────────────────┴──────────────────┐
        ▼                                     ▼
   ┌─────────┐                          ┌─────────┐
   │ PREDICT │                          │ UPDATE  │
   │  Phase  │                          │  Phase  │
   └─────────┘                          └─────────┘
   Use IMU (100Hz)                      Use DVL/GPS/Depth
   Propagate forward                    Correct drift
        │                                     │
        └─────────────► x(t+1) ◄──────────────┘
```

---

## Two Phases Explained

### **Phase 1: PREDICT** (Time Update)
Uses **IMU** at 100 Hz to propagate state forward.

**Input:** IMU reading (ax, ay, az, gx, gy, gz)

**Process:**
```
1. Rotate accelerations to navigation frame
   a_nav = R_bn · a_body - [0, 0, g]

2. Update velocity
   v_new = v_old + a_nav · dt

3. Update position  
   p_new = p_old + v_nav · dt

4. Update attitude
   η_new = η_old + W(φ,θ) · ω_body · dt

5. Update covariance (linearize)
   P⁻ = F · P · Fᵀ + Q
```

**Result:** State prediction x⁻ and covariance P⁻

---

### **Phase 2: UPDATE** (Measurement Update)
Uses **DVL/GPS/Depth/Mag** to correct prediction.

**Input:** Sensor measurement z

**Process:**
```
1. Innovation (measurement - prediction)
   y = z - h(x⁻)

2. Innovation covariance
   S = H · P⁻ · Hᵀ + R

3. Kalman Gain (optimal blend)
   K = P⁻ · Hᵀ · S⁻¹

4. State correction
   x⁺ = x⁻ + K · y

5. Covariance correction (Joseph form)
   P⁺ = (I - K·H) · P⁻ · (I - K·H)ᵀ + K·R·Kᵀ
```

**Result:** Corrected state x⁺ and covariance P⁺

---

## Simple Example with Real Numbers

### Setup (t = 0.32s)
```
Current state:
  Position: px = -1.30 m, py = 1.80 m, pz = 0.09 m
  Velocity: vx = 0.10 m/s, vy = 0.05 m/s, vz = 0.02 m/s
  Attitude: roll = -179°, pitch = 0.4°, yaw = 17.4°

Time step: dt = 0.01 s
```

---

### PREDICT Phase

**IMU Input:**
```
Accelerometer: ax=0.12, ay=-0.03, az=9.85 m/s²
Gyroscope:     gx=-0.001, gy=0.025, gz=-0.002 rad/s
```

**Calculations:**
```
1. Rotate acceleration to nav frame
   a_nav = [0.105, -0.051, 0.039] m/s²
   
2. Update velocity
   vx = 0.10 + 0.105×0.01 = 0.101 m/s ✓
   vy = 0.05 - 0.051×0.01 = 0.049 m/s ✓
   vz = 0.02 + 0.039×0.01 = 0.020 m/s ✓

3. Update position (using old velocity)
   px = -1.30 + 0.11×0.01 = -1.299 m ✓
   py = 1.80 + 0.02×0.01 = 1.800 m ✓
   pz = 0.09 + 0.02×0.01 = 0.090 m ✓

4. Update attitude (using gyros)
   roll = -179.0° + (-0.001°) ≈ -179.0° ✓
   pitch = 0.4° + 0.014° = 0.41° ✓
   yaw = 17.4° - 0.001° ≈ 17.4° ✓
```

**Predicted State:** x⁻ = [-1.299, 1.800, 0.090, 0.101, 0.049, 0.020, -179°, 0.41°, 17.4°]

---

### UPDATE Phase (DVL arrives)

**DVL Measurement:**
```
z_dvl = [0.10, 0.05, 0.02]ᵀ m/s  (true body velocity)
```

**Calculations:**
```
1. Innovation
   y = z - x⁻
     = [0.10, 0.05, 0.02] - [0.101, 0.049, 0.020]
     = [-0.001, 0.001, 0.000] m/s
   (Small error! Prediction was good)

2. Kalman Gain
   K ≈ 0.99  (High gain = trust measurement)
   
3. Correction
   vx = 0.101 + 0.99×(-0.001) = 0.100 m/s ✓
   vy = 0.049 + 0.99×(0.001)  = 0.050 m/s ✓
   vz = 0.020 + 0.99×(0.000)  = 0.020 m/s ✓
```

**Updated State:** x⁺ = [-1.299, 1.800, 0.090, **0.100**, **0.050**, **0.020**, -179°, 0.41°, 17.4°]

---

## Key Formulas (Thesis Defense)

### State Dynamics (Nonlinear)
```
ṗ = R_bn(η) · v_body
v̇ = R_bn(η) · a_body - [0, 0, g]ᵀ
η̇ = W(φ,θ) · ω_body
```

### Kalman Filter Equations
```
PREDICT:
  x⁻ₖ = f(xₖ₋₁, uₖ)           # Nonlinear propagation
  P⁻ₖ = Fₖ Pₖ₋₁ Fₖᵀ + Qₖ      # Linearized covariance

UPDATE:
  yₖ = zₖ - h(x⁻ₖ)             # Innovation
  Sₖ = Hₖ P⁻ₖ Hₖᵀ + Rₖ         # Innovation covariance
  Kₖ = P⁻ₖ Hₖᵀ Sₖ⁻¹            # Kalman gain
  xₖ = x⁻ₖ + Kₖ yₖ              # State update
  Pₖ = (I - Kₖ Hₖ) P⁻ₖ (...)  # Covariance update (Joseph)
```

### Rotation Matrix (ZYX Euler)
```
R_bn = Rz(ψ) Ry(θ) Rx(φ)

     = [cψcθ   cψsθsφ-sψcφ   cψsθcφ+sψsφ]
       [sψcθ   sψsθsφ+cψcφ   sψsθcφ-cψsφ]
       [-sθ    cθsφ          cθcφ        ]
```

---

## Sensor Update Rates

| Sensor | Rate | Measures | Update Frequency |
|--------|------|----------|------------------|
| **IMU** | 100 Hz | ax,ay,az,gx,gy,gz | Every cycle (PREDICT) |
| **DVL** | 10 Hz | vx,vy,vz | Every 10th cycle (UPDATE) |
| **Depth** | 2 Hz | pz | Every 50th cycle (UPDATE) |
| **GPS** | 1 Hz | px,py | Every 100th cycle (UPDATE) |
| **Mag** | 5 Hz | yaw | Every 20th cycle (UPDATE) |

---

## Kalman Gain Interpretation

The Kalman gain K determines **how much to trust the measurement** vs prediction:

```
K ≈ 1.0  →  Trust measurement completely
             (sensor very accurate OR prediction very uncertain)

K ≈ 0.5  →  Balance measurement and prediction equally
             (sensor and prediction have similar uncertainty)

K ≈ 0.0  →  Trust prediction, ignore measurement
             (sensor very noisy OR prediction very confident)
```

**Mathematical:**
```
K = P⁻ Hᵀ / (H P⁻ Hᵀ + R)

If R → 0 (perfect sensor):     K → 1 (full trust)
If P⁻ → 0 (perfect prediction): K → 0 (ignore sensor)
```

---

## Uncertainty Evolution

```
          Prediction         Update
            (grow)          (shrink)
               │               │
    ┌──────────▼───────────────▼──────────┐
    │  σ increases    →    σ decreases    │
    │  (add Q)               (correct)     │
    └──────────────────────────────────────┘
         ▲                         │
         └─────────┬───────────────┘
                   │
              Next cycle
```

Over many cycles, uncertainty **converges to steady state** where growth from Q balances reduction from measurements.

**Our result:** σ_pos converges to 0.089 m (excellent!)

---

## Common Questions

### **Q: Why does position uncertainty stay low even without GPS for seconds?**
**A:** DVL provides velocity updates every 0.1s. Position uncertainty grows slowly:
```
σ_pos² = σ_vel² · t²
σ_pos = 0.007 m/s · 1s = 0.007 m (after 1 second)
```
So even 1 second without GPS, position only drifts 7mm!

### **Q: What if all sensors fail?**
**A:** The filter continues predicting using IMU, but uncertainty grows:
```
After 10s: σ_pos ≈ 0.07 m (still good!)
After 60s: σ_pos ≈ 0.4 m (acceptable)
After 300s: σ_pos ≈ 2 m (poor)
```

### **Q: How do you know the filter is working correctly?**
**A:** Check **innovation consistency** (whiteness test):
```
yₖ ~ N(0, Sₖ)  (innovations should be zero-mean Gaussian)

If yₖ >> σ_S → Filter too confident (R or Q too small)
If yₖ << σ_S → Filter not confident enough (R or Q too large)
```

---

## Tuning Parameters (Quick Reference)

| Parameter | Effect | Too Small | Too Large |
|-----------|--------|-----------|-----------|
| **R** (sensor noise) | Trust in measurements | Filter jumpy, noisy | Ignores good data, slow convergence |
| **Q** (process noise) | Trust in prediction | Filter overconfident, diverges | Filter uncertain, oscillates |
| **P₀** (initial cov) | Starting uncertainty | Fast convergence (if accurate) | Slow convergence (safe) |

**Our optimized values:**
- R_gps = 0.5² = 0.25 m²
- R_dvl = 0.02² = 0.0004 m²/s²
- Q_base = 0.01² = 0.0001

---

## The "Aha!" Moment

**Before optimization:**
```
σ_gps = 1.5 m → K_gps ≈ 0.5 → Barely trusts GPS
σ_proc = 0.05 → Q large → State drifts quickly

Result: Oscillates between drift and corrections
        Uncertainty stays high (~2m)
```

**After optimization:**
```
σ_gps = 0.5 m → K_gps ≈ 0.9 → Strongly trusts GPS
σ_proc = 0.01 → Q small → State stable between measurements

Result: Smooth tracking of sensors
        Uncertainty converges (~0.09m)
```

---

## For Your Defense/Presentation

**30-second explanation:**

"The EKF fuses multiple sensors optimally. Every 10ms, the IMU predicts state forward using physics. Every 100ms, DVL corrects velocity drift. Every 1 second, GPS corrects position drift. The Kalman gain automatically computes the optimal blend based on sensor accuracies. By tuning noise parameters to match real sensor specs, we achieve 0.089m position accuracy—22 times better than before."

**Show this slide:**
```
┌────────────────────────────────────────────┐
│  IMU (100 Hz)  →  Smooth predictions       │
│  DVL (10 Hz)   →  Correct velocity drift   │
│  GPS (1 Hz)    →  Correct position drift   │
│  ──────────────────────────────────────    │
│  EKF: Optimal sensor fusion                │
│  Result: 0.089m accuracy (sub-decimeter!)  │
└────────────────────────────────────────────┘
```

---

**Quick reference card for your pocket! 🎯**
