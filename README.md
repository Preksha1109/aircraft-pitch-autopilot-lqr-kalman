# Aircraft Pitch Autopilot using LQR and Kalman Filter

## Overview
This project implements a longitudinal aircraft pitch control system using modern control techniques. A linearized state-space model of aircraft dynamics is used to design an LQR controller for pitch angle tracking, combined with a Kalman filter for state estimation in the presence of sensor noise.

The system is validated through MATLAB/Simulink simulations, analysing stability, transient response, and robustness to disturbances.

## Objectives
- Model longitudinal aircraft dynamics in state-space form
- Design an optimal state-feedback controller (LQR)
- Implement a Kalman filter for noisy sensor measurements
- Evaluate system performance through simulation

## System Model
The longitudinal dynamics are represented as:

x' = Ax + Bu  
y = Cx + Du  

Where:
- x = [u, w, q, θ]ᵀ (velocity, vertical velocity, pitch rate, pitch angle)
- u = elevator deflection

The model is linearized around a steady-level flight condition.

## Control Design
An LQR controller is designed by minimizing the cost function:

J = ∫ (xᵀQx + uᵀRu) dt

The optimal gain matrix K is obtained by solving the Riccati equation.

## State Estimation
A Kalman filter is implemented to estimate system states under measurement noise and process disturbances.

## Tools Used
- MATLAB
- Simulink
- Control System Toolbox

## Results
- Stable closed-loop pitch response
- Improved tracking performance
- Robustness to sensor noise

## Future Work
- Extend to full 6-DOF aircraft model
- Implement nonlinear dynamics
- Test different flight conditions
