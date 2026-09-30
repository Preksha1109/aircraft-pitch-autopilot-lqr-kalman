# Quick Start Guide

## Installation (1 minute)

1. Extract the zip file to your MATLAB workspace

2. Open MATLAB and navigate to the project directory:
   ```matlab
   cd aircraft_pitch_autopilot
   ```

3. Run the setup script:
   ```matlab
   SETUP
   ```
   This checks dependencies and configures paths.

## Run (2 minutes)

Execute the complete pipeline:
```matlab
run_autopilot
```

This will:
- Build the aircraft model
- Design the LQR controller
- Design the Kalman filter
- Run a closed-loop simulation
- Generate 11 plots showing results

## View Results (1 minute)

Open the generated plots in `figs/` directory:
- `06_pitch_angle_tracking.png` → Main performance
- `09_control_input.png` → Elevator response
- `11_performance_summary.png` → Overall dashboard

## Tune the Controller (optional)

**Make it respond faster:**
1. Open `matlab/lqr_controller.m`
2. Change line: `q_theta = 200;` (was 100)
3. Run: `lqr_controller` (after loading aircraft model)

**Make it smoother (less jerky):**
1. Open `matlab/lqr_controller.m`
2. Change line: `r = 10;` (was 1)
3. Run: `lqr_controller`

**Improve noise rejection:**
1. Open `matlab/kalman_filter.m`
2. Change line: `q_process = 0.01 * eye(4);` (increase value)
3. Run: `kalman_filter`

## Test Robustness

Check sensitivity to parameter variations:
```matlab
load('aircraft_model.mat');
load('lqr_controller.mat');
load('kalman_filter.mat');
analyze_robustness
```

## Validate Design

Run automated test suite:
```matlab
load('aircraft_model.mat');
load('lqr_controller.mat');
load('kalman_filter.mat');
test_suite
```

All 12 tests should pass (✓).

## Next Steps

- **Modify aircraft**: Edit parameters in `matlab/aircraft_model.m`
- **Add new disturbance**: Edit `simulate_autopilot.m` (look for "wind gust")
- **Change reference command**: Edit `sim.theta_ref` in `matlab/parameters.m`
- **Implement in Simulink**: Use the models saved as `.mat` files

## Common Issues

| Problem | Solution |
|---------|----------|
| "Undefined function 'lqr'" | Install Control System Toolbox (Home → Add-Ons) |
| Plots not saving | Check `figs/` directory has write permissions |
| Simulation too slow | Reduce `sim.t_final` in `matlab/parameters.m` |
| Controller oscillating | Increase `r` in `lqr_controller.m` (more conservative) |

## File Reference

| File | Purpose |
|------|---------|
| `run_autopilot.m` | Master script (start here) |
| `aircraft_model.m` | Define aircraft dynamics |
| `lqr_controller.m` | Design optimal controller |
| `kalman_filter.m` | Design state estimator |
| `simulate_autopilot.m` | Run closed-loop simulation |
| `parameters.m` | All tunable constants |
| `test_suite.m` | Validation tests |

## Performance Expectations

For the default configuration:

- **Rise time**: ~0.8 seconds (10° step)
- **Settling time**: ~3.2 seconds (5% band)
- **Overshoot**: <5%
- **Steady-state error**: <0.01°
- **Robustness**: Stable under ±20% aerodynamic uncertainty

## Contact & Support

For control theory questions, see:
- README.md → "Control Theory Background"
- Comments in source code
- References section in README.md

---

**That's it!** You now have a working aircraft pitch autopilot. 🚁
