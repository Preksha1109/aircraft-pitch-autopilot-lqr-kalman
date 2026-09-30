%% Kalman Filter Design for Aircraft State Estimation
% Estimates the full state vector [u, w, q, theta] from noisy sensor measurements
% of pitch angle, pitch rate, and vertical velocity
%
% Measurement model: z = Cx + v, where v ~ N(0, R_kf)
% Process model: x' = Ax + w, where w ~ N(0, Q_kf)

clear all; close all; clc;

% Load aircraft model and LQR controller
load('aircraft_model.mat');
load('lqr_controller.mat');

fprintf('=== Kalman Filter Design ===\n\n');

%% Sensor noise characteristics
% Typical sensor errors for small aircraft

% IMU: MEMS accelerometer and gyroscope
% Pitch rate sensor (gyro): ~0.1 deg/s noise std
gyro_noise = deg2rad(0.1);      % pitch rate measurement noise (rad/s)

% Angle of attack vane / vertical velocity estimate
% Can be derived from airspeed and vertical speed sensors
% Typical: ~0.05 m/s noise std
w_noise = 0.05;                 % vertical velocity measurement noise (m/s)

% Pitch angle: inferred from integration of gyro or from attitude reference
% Can also use air data computer or GPS/INS
% Typical: ~1 deg integration drift
theta_noise = deg2rad(1);       % pitch angle measurement noise (rad)

% Measurement noise covariance matrix (diagonal)
R_kf = diag([theta_noise^2, gyro_noise^2, w_noise^2]);

fprintf('Measurement Noise Covariance R:\n');
fprintf('  θ std dev:  %.4f rad (%.2f deg)\n', theta_noise, rad2deg(theta_noise));
fprintf('  q std dev:  %.4f rad/s (%.2f deg/s)\n', gyro_noise, rad2deg(gyro_noise));
fprintf('  w std dev:  %.4f m/s\n', w_noise);
disp(R_kf);

%% Process noise characteristics
% Model uncertainty and atmospheric turbulence
% Represent as white noise in state equations

% Turbulence: typically small
% Model error: assume ~10% of control effectiveness
q_process = 0.001 * eye(4);    % small process noise (tuning parameter)

fprintf('\nProcess Noise Covariance Q:\n');
disp(q_process);

%% Steady-state Kalman filter gain
% For continuous-time system: P = lqe solves the dual problem
% (equivalent to LQR but for observer)

[L, P_kf, evals_obs] = lqe(A, eye(4), C, q_process, R_kf);

fprintf('\n=== Kalman Filter Gains ===\n');
fprintf('Observer gain L (4x3):\n');
disp(L);

% Interpretation
fprintf('\nFilter state update:\n');
fprintf('  x_hat_dot = Ax_hat + Bu - L(Cx_hat - z)\n');
fprintf('  dx_hat = (A - LC)x_hat + Lu + Lz\n');

% Observer (filter) eigenvalues
fprintf('\nObserver eigenvalues:\n');
for i = 1:length(evals_obs)
    if imag(evals_obs(i)) == 0
        fprintf('  λ%d = %.4f (real)\n', i, real(evals_obs(i)));
    else
        fprintf('  λ%d = %.4f ± j%.4f\n', i, real(evals_obs(i)), abs(imag(evals_obs(i))));
    end
end

% Observer settling time
[wn_obs, zeta_obs, ~] = damp(A - L*C);
ts_obs = max(3./(zeta_obs.*wn_obs));
fprintf('\nEstimated observer settling time (5%% criterion): %.2f sec\n', ts_obs);

%% Combined observer-controller system
% Full state feedback using estimated states
% u = -K*x_hat
%
% Closed-loop with estimation:
% x_dot = (A - BK)x + BK(x - x_hat)
% x_hat_dot = (A - LC)x_hat + Bu + Lz
%             = (A - LC - BK)x_hat + BK*x + Lz

% In error coordinates: e = x - x_hat
% e_dot = (A - LC)e

% Separation principle: controller and observer poles are independent

A_obs = A - L*C;  % Observer dynamics
fprintf('\n=== Observer-Controller Architecture ===\n');
fprintf('Separation principle applies:\n');
fprintf('  • Controller poles: LQR poles (see lqr_controller.mat)\n');
fprintf('  • Observer poles: Kalman filter poles (above)\n');
fprintf('  • Combined system poles: union of both sets\n');

%% Measurement matrix and observability
fprintf('\n=== Observability Analysis ===\n');
rank_obs = rank(obsv(A, C));
fprintf('Observability matrix rank: %d (full rank = 4)\n', rank_obs);

if rank_obs == 4
    fprintf('✓ System is observable - all states can be estimated\n');
else
    fprintf('✗ Warning: System not fully observable!\n');
end

%% Simulation parameters for closed-loop with filter
fprintf('\n=== Recommended Simulation Parameters ===\n');
fprintf('Integration time step: dt ≤ %.4f sec (10× fastest freq)\n', 0.1/max(abs(real(evals_obs))));
fprintf('Simulation duration: 30-60 sec (to see settling)\n');
fprintf('Sensor update rate: 50-100 Hz typical for small aircraft\n');

%% Save Kalman filter
save('kalman_filter.mat', 'L', 'P_kf', 'R_kf', 'q_process', 'A_obs');
fprintf('\nKalman filter saved to kalman_filter.mat\n');

%% Plot filter poles
figure('Name', 'Observer Pole Map');
hold on;
plot(real(evals_obs), imag(evals_obs), 'gs', 'MarkerSize', 10, 'DisplayName', 'Observer poles');
grid on;
xlabel('Real axis (1/s)');
ylabel('Imaginary axis (rad/s)');
title('Kalman Filter Observer Pole Locations');
legend;
axis equal;
xlim([-2 0.5]); ylim([-2 2]);

savefig('figs/05_observer_poles.fig');
print('figs/05_observer_poles.png', '-dpng', '-r150');
close();

fprintf('\nObserver pole plot saved.\n');
