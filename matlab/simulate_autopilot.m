%% Aircraft Pitch Autopilot Simulation
% Closed-loop simulation with:
%  • LQR controller
%  • Kalman filter for state estimation
%  • Sensor noise
%  • Control saturation
%  • Disturbance (wind gust)

clear all; close all; clc;

% Load all models and controllers
load('aircraft_model.mat');
load('lqr_controller.mat');
load('kalman_filter.mat');

fprintf('=== Aircraft Pitch Autopilot Closed-Loop Simulation ===\n\n');

%% Simulation setup
dt = 0.01;          % integration time step (100 Hz)
t_final = 60;       % simulation duration (seconds)
t = 0:dt:t_final;
N = length(t);

% Initial conditions
x0 = [0; 0; 0; 0];           % aircraft starts at trim
x_hat0 = [0; 0; 0; 0];       % filter initial estimate (same as truth)

% Reference command
theta_ref = deg2rad(10);      % 10 degree pitch angle command at t=5s
t_cmd = 5;

% Disturbance: wind gust (horizontal, affects vertical velocity)
w_gust_mag = 1;              % 1 m/s gust
t_gust_start = 15;
t_gust_end = 17;

%% Preallocate storage
x_true = zeros(4, N);
x_est = zeros(4, N);
x_error = zeros(4, N);
z_meas = zeros(3, N);
u_cmd = zeros(1, N);
u_sat = zeros(1, N);

x_true(:, 1) = x0;
x_est(:, 1) = x_hat0;

%% RK4 integrator function
function x_next = rk4_step(x, u, A, B, dt)
    k1 = A*x + B*u;
    k2 = A*(x + 0.5*dt*k1) + B*u;
    k3 = A*(x + 0.5*dt*k2) + B*u;
    k4 = A*(x + dt*k3) + B*u;
    x_next = x + (dt/6)*(k1 + 2*k2 + 2*k3 + k4);
end

%% Main simulation loop
fprintf('Simulating %d steps (%.1f seconds)...\n', N, t_final);

for k = 1:N-1
    %% Reference command
    if t(k) >= t_cmd
        cmd_active = 1;
    else
        cmd_active = 0;
    end
    
    %% Disturbance (wind gust)
    if t(k) >= t_gust_start && t(k) <= t_gust_end
        w_gust = w_gust_mag;
    else
        w_gust = 0;
    end
    
    %% Measurement (with noise)
    % Simulated sensors measure pitch angle, pitch rate, vertical velocity
    theta_meas = C(1,:)*x_true(:,k) + theta_noise * randn();
    q_meas = C(2,:)*x_true(:,k) + gyro_noise * randn();
    w_meas = C(3,:)*x_true(:,k) + w_noise * randn();
    z_meas(:, k) = [theta_meas; q_meas; w_meas];
    
    %% Kalman filter state estimation
    % Measurement update (correct)
    z_pred = C * x_est(:, k);
    innovation = z_meas(:, k) - z_pred;
    x_est(:, k) = x_est(:, k) + L * innovation;
    
    %% LQR control law
    % u = -K*(x_est - x_ref) = -K*x_est + K*x_ref
    % With reference: u = -K*x_est + K*[x_ref; 0; 0; theta_ref]
    
    x_ref = [0; 0; 0; theta_ref * cmd_active];
    u_unsaturated = -K * x_est(:, k) + K * x_ref;
    
    % Saturation (elevator deflection limits: ±25 deg)
    u_sat(:, k) = max(min(u_unsaturated, delta_e_max), -delta_e_max);
    
    % Store unsaturated command for reference
    u_cmd(:, k) = u_unsaturated;
    
    %% Aircraft state propagation (with disturbance)
    % Add wind gust to vertical velocity equation
    B_gust = B;
    B_gust(:,1) = B(:,1);  % elevator
    
    % Wind gust enters as process noise
    % w_dot affected by gust: w_dot = Zu*u + Zw*w + Zq*q - g*cos(theta) + w_gust_rate
    % Simplified: treat as state disturbance
    
    dx_true = A*x_true(:,k) + B*u_sat(:,k);
    
    % Add process noise (turbulence) to vertical velocity state
    dx_true(2) = dx_true(2) + w_gust;
    
    x_true(:,k+1) = rk4_step(x_true(:,k), u_sat(:,k), A, B, dt) + [0; w_gust*dt; 0; 0];
    
    %% Kalman filter time update (predict)
    x_est(:,k+1) = rk4_step(x_est(:,k), u_sat(:,k), A, B, dt);
    
    %% Estimation error
    x_error(:,k) = x_true(:,k) - x_est(:,k);
end

% Final error
x_error(:,N) = x_true(:,N) - x_est(:,N);

fprintf('Simulation complete.\n\n');

%% Performance metrics
fprintf('=== Performance Metrics ===\n\n');

% Pitch angle response to step command
theta_true = x_true(4,:) * rad2deg(1);
theta_est = x_est(4,:) * rad2deg(1);
theta_error = x_error(4,:) * rad2deg(1);

% Find rise time (10% to 90%)
idx_90 = find(theta_est >= 0.9*10, 1);
idx_10 = find(theta_est >= 0.1*10, 1);
if ~isempty(idx_90) && ~isempty(idx_10)
    rise_time = (idx_90 - idx_10)*dt;
    fprintf('Rise time (10%%-90%%): %.2f sec\n', rise_time);
end

% Settling time (within 5% of steady state after command)
if ~isempty(idx_90)
    idx_settle = find(abs(theta_est(idx_90:end) - 10) <= 0.5, 1);  % 5% of 10 deg
    if ~isempty(idx_settle)
        settle_time = (idx_90 + idx_settle - 1)*dt;
        fprintf('Settling time (5%% band): %.2f sec\n', settle_time);
    end
end

% Steady-state error
ss_error = abs(theta_est(end) - 10);
fprintf('Steady-state pitch angle error: %.4f deg\n', ss_error);

% Overshoot
if ~isempty(idx_90)
    theta_peak = max(theta_est(idx_90:end));
    overshoot = max(0, (theta_peak - 10)/10 * 100);
    fprintf('Overshoot: %.2f %%\n', overshoot);
end

% Estimation error
rms_theta_error = sqrt(mean(theta_error(t_cmd/dt:end).^2));
max_theta_error = max(abs(theta_error(t_cmd/dt:end)));
fprintf('\nEstimation Error (after command):\n');
fprintf('  RMS pitch angle error: %.4f deg\n', rms_theta_error);
fprintf('  Max pitch angle error: %.4f deg\n', max_theta_error);

% Control effort
elevator_deg = u_sat * rad2deg(1);
rms_elevator = sqrt(mean(elevator_deg(t_cmd/dt:end).^2));
fprintf('\nControl Effort:\n');
fprintf('  RMS elevator deflection: %.2f deg\n', rms_elevator);
fprintf('  Max elevator deflection: %.2f deg\n', max(abs(elevator_deg)));

%% Save simulation results
save('simulation_results.mat', 't', 'x_true', 'x_est', 'x_error', 'z_meas', ...
     'u_cmd', 'u_sat', 'theta_true', 'theta_est', 'theta_error');

fprintf('\nResults saved to simulation_results.mat\n');

%% Plotting
plot_simulation_results;
