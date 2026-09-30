%% Aircraft Pitch Autopilot - Complete Pipeline
% Master script to run the entire design and simulation
%
% Steps:
% 1. Define parameters
% 2. Build aircraft model
% 3. Design LQR controller
% 4. Design Kalman filter
% 5. Run closed-loop simulation
% 6. Analyze results

clc; clear all; close all;

fprintf('\n');
fprintf('╔══════════════════════════════════════════════════════════════╗\n');
fprintf('║  Aircraft Pitch Autopilot with LQR and Kalman Filter        ║\n');
fprintf('║  Design and Simulation Pipeline                             ║\n');
fprintf('╚══════════════════════════════════════════════════════════════╝\n\n');

% Create output directories
if ~exist('figs', 'dir')
    mkdir('figs');
end

%% Step 1: Parameters
fprintf('Step 1: Loading parameters...\n');
parameters;

%% Step 2: Aircraft model
fprintf('\nStep 2: Building aircraft longitudinal dynamics model...\n');
fprintf('────────────────────────────────────────────────────────────────\n');
aircraft_model;
fprintf('────────────────────────────────────────────────────────────────\n');

%% Step 3: LQR controller design
fprintf('\nStep 3: Designing LQR controller...\n');
fprintf('────────────────────────────────────────────────────────────────\n');
lqr_controller;
fprintf('────────────────────────────────────────────────────────────────\n');

%% Step 4: Kalman filter design
fprintf('\nStep 4: Designing Kalman filter...\n');
fprintf('────────────────────────────────────────────────────────────────\n');
kalman_filter;
fprintf('────────────────────────────────────────────────────────────────\n');

%% Step 5: Simulation
fprintf('\nStep 5: Running closed-loop simulation...\n');
fprintf('────────────────────────────────────────────────────────────────\n');
simulate_autopilot;
fprintf('────────────────────────────────────────────────────────────────\n');

%% Step 6: Analysis
fprintf('\n');
fprintf('╔══════════════════════════════════════════════════════════════╗\n');
fprintf('║                    Design Complete                          ║\n');
fprintf('╚══════════════════════════════════════════════════════════════╝\n\n');

fprintf('Outputs:\n');
fprintf('  • Models saved: aircraft_model.mat, lqr_controller.mat, kalman_filter.mat\n');
fprintf('  • Simulation results: simulation_results.mat\n');
fprintf('  • Plots: figs/*.png (11 figures)\n\n');

fprintf('Next steps:\n');
fprintf('  1. Review plots in figs/ directory\n');
fprintf('  2. Tune LQR gains (Q, R) in lqr_controller.m if needed\n');
fprintf('  3. Tune Kalman filter (R_kf, Q_kf) in kalman_filter.m if needed\n');
fprintf('  4. Run analyze_robustness.m for sensitivity analysis\n\n');

fprintf('Implementation notes:\n');
fprintf('  • Elevator saturation: ±25 degrees\n');
fprintf('  • Sensor noise: gyro 0.1°/s, pitch 1°, vertical velocity 0.05 m/s\n');
fprintf('  • Wind gust injected at t=15-17s (1 m/s)\n');
fprintf('  • Simulation time step: 100 Hz\n\n');
