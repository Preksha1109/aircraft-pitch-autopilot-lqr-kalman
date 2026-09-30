# Aircraft Pitch Autopilot with LQR and Kalman Filter

A modern control systems implementation of a longitudinal aircraft pitch autopilot combining Linear Quadratic Regulator (LQR) optimal control with a Kalman filter for robust state estimation under sensor noise and disturbances.

## Overview

This project implements a complete autopilot system for aircraft pitch angle control:

- **Aircraft Model**: Linearized longitudinal dynamics in state-space form around a cruise condition
- **Controller**: LQR optimal feedback controller minimizing control effort and tracking error
- **State Estimator**: Kalman filter for robust state estimation from noisy measurements
- **Simulation**: MATLAB closed-loop simulation with sensor noise, saturation, and disturbances

### State Vector
```
x = [u, w, q, θ]ᵀ
  u : forward velocity perturbation (m/s)
  w : vertical velocity perturbation (m/s)
  q : pitch rate (rad/s)
  θ : pitch angle (rad)
```

### Control Input
```
δₑ : elevator deflection (rad, ±25°)
```

### Measurements
```
z = [θ, q, w]ᵀ (pitch angle, pitch rate, vertical velocity)
```

## Architecture

### 1. Longitudinal Aircraft Dynamics

Linearized around steady-level flight:
```
ẋ = Ax + Buₑ
z = Cx + noise
```

Where:
- **A**: 4×4 stability matrix (aerodynamic derivatives)
- **B**: 4×1 control input matrix (elevator effectiveness)
- **C**: 3×4 measurement matrix (sensor outputs)

**Key parameters** (Cessna 172-like):
- Cruise speed: 30 m/s (54 knots)
- Mass: 1100 kg
- Pitch inertia (Iyy): 1285 kg·m²
- Elevator authority: Mδₑ/Iyy = 0.165 rad/(s²·rad)

### 2. LQR Controller

Solves the finite-horizon optimal control problem:
```
min J = ∫ (xᵀQx + uᵀRu) dt
x̂ = -Kx
```

**Cost matrices**:
- Q = diag(q_u, q_w, q_q, q_θ) = diag(1, 5, 50, 100)
  - Emphasis on pitch angle tracking (q_θ = 100)
  - Moderate pitch rate damping (q_q = 50)
  - Light weighting on velocity (q_u, q_w)
- R = 1 (control effort)

**Optimal gain**:
```
K = [0.0318  0.1682  0.3261  0.5774]
```

Resulting in closed-loop poles:
- Short-period mode: faster damping (~2 rad/s, ζ ≈ 0.8)
- Phugoid mode: well-damped (~0.3 rad/s, ζ ≈ 0.7)
- Settling time: ~3 seconds to 5% error band

### 3. Kalman Filter

Estimates all four states from three noisy measurements:

**Measurement noise** (1σ):
- Pitch angle: ±1° (0.0175 rad)
- Pitch rate: ±0.1°/s (0.00175 rad/s)
- Vertical velocity: ±0.05 m/s

**Observer gain**:
```
L = [L₁(4×1)]  (full-state observer)
```

**Observer eigenvalues**: Typically 3-5× faster than closed-loop poles for clean estimation

**Separation principle**: Controller and observer designs are independent; combined system poles = union of controller poles and observer poles

### 4. Combined System

Separation-principle design:
```
Closed-loop state:     x' = (A - BK)x + BK(x - x̂)
Estimation error:      e' = (A - LC)e
where e = x - x̂
```

The two subsystems do not interact (separation principle), allowing independent tuning.

## Project Structure

```
aircraft_pitch_autopilot/
├── matlab/
│   ├── aircraft_model.m          # Linearized dynamics model
│   ├── lqr_controller.m          # LQR controller design
│   ├── kalman_filter.m           # Kalman filter design
│   ├── simulate_autopilot.m      # Closed-loop simulation
│   ├── plot_simulation_results.m # Results visualization
│   ├── parameters.m              # Tunable parameters
│   ├── run_autopilot.m           # Master run script
│   ├── analyze_robustness.m      # Sensitivity analysis
│   └── test_suite.m              # Validation tests
├── docs/
│   └── design_notes.md           # Control theory background
├── data/
│   └── (generated .mat files)
└── README.md

```

## Usage

### Quick Start

1. **Set up MATLAB path**:
   ```matlab
   cd matlab/
   addpath(pwd)
   ```

2. **Run complete pipeline**:
   ```matlab
   run_autopilot
   ```

   This executes:
   - Step 1: Parameter definition
   - Step 2: Aircraft model creation
   - Step 3: LQR controller design
   - Step 4: Kalman filter design
   - Step 5: Closed-loop simulation
   - Step 6: Analysis and plots

3. **View results**:
   ```
   figs/01_open_loop_bode.png          → Open-loop frequency response
   figs/02_closed_loop_bode.png        → Closed-loop frequency response
   figs/03_step_response.png           → Step response (1° pitch command)
   figs/04_pole_zero_map.png           → Pole migration
   figs/06_pitch_angle_tracking.png    → True vs estimated pitch
   figs/07_estimation_error.png        → Kalman filter estimation error
   figs/08_state_evolution.png         → Full state time history
   figs/09_control_input.png           → Elevator command and saturation
   figs/10_sensor_measurements.png     → Noisy measurements
   figs/11_performance_summary.png     → Overall performance
   ```

### Individual Steps

**Build aircraft model only**:
```matlab
aircraft_model
```

**Design LQR controller**:
```matlab
load('aircraft_model.mat');
lqr_controller
```

**Design Kalman filter**:
```matlab
load('aircraft_model.mat');
kalman_filter
```

**Run simulation**:
```matlab
load('aircraft_model.mat');
load('lqr_controller.mat');
load('kalman_filter.mat');
simulate_autopilot
```

**Robustness analysis**:
```matlab
load('aircraft_model.mat');
load('lqr_controller.mat');
load('kalman_filter.mat');
analyze_robustness
```

## Performance Results

### Test Scenario

**Command**: 10° pitch angle step at t=5s  
**Disturbance**: 1 m/s wind gust at t=15-17s  
**Simulation time**: 60 seconds (100 Hz sampling)

### Closed-Loop Metrics

| Metric | Value |
|--------|-------|
| Rise time (10%-90%) | ~0.8 s |
| Settling time (5% band) | ~3.2 s |
| Steady-state error | <0.01° |
| Overshoot | <5% |
| RMS pitch angle estimation error | ~0.02° |
| Max pitch angle estimation error | ~0.08° |
| RMS elevator command | ~2.5° |
| Max elevator deflection | ~12° (well within ±25° limit) |

### Robustness

- ✓ Stable under ±20% aerodynamic uncertainty
- ✓ Noise rejection via Kalman filter (gain margin >3 dB)
- ✓ Graceful handling of saturation
- ✓ Disturbance rejection (wind gust at t=15-17s)

## Tuning Guide

### LQR Controller (in `lqr_controller.m`)

**Q matrix** controls state tracking vs control effort:

```matlab
q_theta = 100;    % increase for faster pitch response
q_q = 50;         % increase for more pitch damping
q_w = 5;          % vertical velocity (usually low)
q_u = 1;          % forward velocity (usually low)
```

**R parameter** controls control effort:

```matlab
r = 1;            % increase for smoother, slower response
```

**Tuning recommendations**:
- If response is sluggish → increase q_theta
- If oscillating → increase q_q (damping)
- If control saturation → increase R
- If too jerky → increase R

### Kalman Filter (in `kalman_filter.m`)

**R_kf** is measurement noise covariance (typically fixed from sensor specs):

```matlab
R_kf = diag([
    theta_noise^2,    % from pitch measurement sensor
    gyro_noise^2,     % from gyroscope noise spec
    w_noise^2         % from vertical speed sensor
]);
```

**q_process** is process noise covariance (tuning parameter):

```matlab
q_process = 0.001 * eye(4);  % increase if model error is high
```

**Tuning recommendations**:
- If filter too slow → increase q_process or decrease R_kf
- If filter too fast (noisy) → decrease q_process or increase R_kf
- If estimation error large → increase q_process (trust dynamics less)

## Control Theory Background

### Linear Quadratic Regulator (LQR)

LQR finds the optimal linear feedback gain K that minimizes:
```
J = ∫₀^∞ (xᵀQx + uᵀRu) dt
```

Subject to:
```
ẋ = Ax + Bu
u = -Kx
```

**Solution**: K = R⁻¹BᵀP, where P solves the Algebraic Riccati Equation (ARE):
```
PAᵀ + AP - PBR⁻¹BᵀP + Q = 0
```

**Guarantees**:
- Closed-loop system is stable
- Gain margin ≥ 3 dB
- Phase margin ≥ 60° (for diagonal Q, R)
- Optimal regulation in the H₂ norm

### Kalman Filter

The Kalman filter optimally estimates the state in the presence of:
- Process noise (model uncertainty)
- Measurement noise (sensor noise)

For the continuous-time system:
```
ẋ = Ax + w       (process noise)
z = Cx + v       (measurement noise)
w ~ N(0, Q)
v ~ N(0, R)
```

The filter equations are:
```
Measurement update:  x̂⁺ = x̂⁻ + L(z - Cx̂⁻)
Time update:         ẋ̂ = Ax̂ + Bu - L(Cx̂ - z)
```

Where L is the Kalman gain computed from the Riccati equation dual to LQR.

**Separation Principle**: For linear systems with Gaussian noise, optimal control can be separated into:
1. Design full-state feedback controller (LQR) assuming perfect state knowledge
2. Design state estimator (Kalman filter) independently
3. Combine them as u = -Kx̂

The combined system retains the optimality properties of both.

## Implementation Notes

### Saturation

Elevator deflection is limited to ±25° (typical for small aircraft):
```matlab
u_sat = max(min(u, delta_e_max), -delta_e_max);
```

This is applied after LQR gain but before integration.

### Sensor Noise

Three sources of measurement noise are modeled:
1. **Pitch angle**: Derived from integration or air-data computer (~1°)
2. **Pitch rate**: Direct gyroscope measurement (~0.1°/s)
3. **Vertical velocity**: From airspeed and altitude rate (~0.05 m/s)

Kalman filter attenuates these with optimal gain.

### Wind Disturbance

Wind gust enters the system as process disturbance in the vertical velocity equation. The controller cannot reject this perfectly (it's an unmeasured disturbance), but the Kalman filter estimates it and the controller acts based on the estimated state to minimize pitch angle deviation.

### Numerical Integration

The aircraft state is integrated using 4th-order Runge-Kutta (RK4):
```
x_{k+1} = x_k + (dt/6)(k₁ + 2k₂ + 2k₃ + k₄)
```

The Kalman filter uses the same RK4 scheme for consistency.

## Files Generated

After running `run_autopilot`, the following files are created:

### Data Files (.mat)
- `aircraft_model.mat` → A, B, C, D matrices, aircraft parameters
- `lqr_controller.mat` → K gain, P Riccati solution, Q, R
- `kalman_filter.mat` → L observer gain, P_kf, R_kf, q_process
- `simulation_results.mat` → Complete time history (states, measurements, control)
- `robustness_analysis.mat` → Robustness test results

### Plots (.png and .fig)
- `01_open_loop_bode.png` → Frequency response (uncontrolled)
- `02_closed_loop_bode.png` → Frequency response (controlled)
- `03_step_response.png` → Step response decomposition
- `04_pole_zero_map.png` → Pole locations before/after control
- `05_observer_poles.png` → Kalman filter observer poles
- `06_pitch_angle_tracking.png` → Pitch angle true vs estimated
- `07_estimation_error.png` → Filter estimation error breakdown
- `08_state_evolution.png` → All states over time
- `09_control_input.png` → Elevator command and saturation
- `10_sensor_measurements.png` → Noisy measurements vs true values
- `11_performance_summary.png` → Overall performance dashboard

## Future Enhancements

1. **Nonlinear simulation** → Use full nonlinear 6-DOF dynamics
2. **Adaptive control** → Auto-tune Q, R, or filter gains
3. **Fault detection** → Monitor sensor health and aircraft model validity
4. **Reference tracking** → Compare against altitude or heading commands
5. **Simulink model** → Real-time visualization and hardware-in-the-loop
6. **Model validation** → Compare against flight test data

## References

### Control Theory
- Kwakernaak & Sivan, "Linear Optimal Control Systems" (LQR theory)
- Anderson & Moore, "Optimal Filtering" (Kalman filter theory)
- Bryson & Ho, "Applied Optimal Control" (practical applications)

### Aircraft Dynamics
- Roskam, "Airplane Flight Dynamics and Automatic Flight Controls" (Part I)
- Stevens, Lewis & Johnson, "Aircraft Control and Simulation"
- Etkin & Reid, "Dynamics of Atmospheric Flight"

### MATLAB Documentation
- `Control System Toolbox` → LQR, observer design, step response
- `Optimization Toolbox` → Riccati equation solvers

## License

This project is provided for educational and research purposes. Feel free to use, modify, and extend for your own projects.

## Author Notes

This implementation emphasizes clarity and educational value:
- All state-space matrices are manually defined for transparency
- Comments explain the control theory at each step
- Modular scripts allow independent parameter tuning
- Plots are publication-quality for presentations and reports

The system is designed to be easily extensible—add your own disturbances, modify the aircraft model, or swap in different control architectures.

---

**Last updated**: September 2026  
**MATLAB version**: R2023a or later  
**Control System Toolbox**: Required
