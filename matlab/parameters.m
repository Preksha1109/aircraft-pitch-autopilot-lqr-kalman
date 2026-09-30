%% Global Parameters for Aircraft Pitch Autopilot
% This script defines all tunable parameters in one place

clear all;

%% Aircraft Parameters
% Typical single-engine aircraft (Cessna 172-like)

aircraft.mass = 1100;           % kg
aircraft.V_cruise = 30;         % m/s (54 knots)
aircraft.Iyy = 1285;            % kg·m² (pitch inertia)
aircraft.g = 9.81;              % m/s²

%% Aerodynamic Stability and Control Derivatives
% Linearized around cruise condition, per radian

% Longitudinal force
aircraft.Xu = -0.041;           % X_u / m
aircraft.Xw = 0.0;
aircraft.Xde = 0.0;

% Vertical force
aircraft.Zu = -0.25;            % Z_u / m
aircraft.Zw = -3.2;             % Z_w / m (vertical stiffness)
aircraft.Zq = -0.15;            % Z_q / m (pitch damping)
aircraft.Zde = -0.5;            % Z_de / m (elevator authority)

% Pitching moment
aircraft.Mu = 0.011;            % M_u / Iyy
aircraft.Mw = -0.012;           % M_w / Iyy
aircraft.Mq = -0.45;            % M_q / Iyy (pitch rate damping)
aircraft.Mde = 0.165;           % M_de / Iyy (elevator moment arm)

% Trim
aircraft.theta0 = 0;            % pitch angle at trim (rad)
aircraft.alpha0 = deg2rad(3);   % angle of attack at trim

%% LQR Controller Tuning
% Cost function: min J = ∫ (x'Qx + u'Ru) dt

lqr.q_theta = 100;              % weight on pitch angle (critical)
lqr.q_q = 50;                   % weight on pitch rate
lqr.q_w = 5;                    % weight on vertical velocity
lqr.q_u = 1;                    % weight on forward velocity
lqr.r = 1;                      % control effort weight

% Saturation limits
lqr.delta_e_max = deg2rad(25);  % elevator deflection (rad)

%% Kalman Filter Tuning
% Measurement and process noise

% Sensor noise standard deviations
kf.theta_noise = deg2rad(1);    % pitch angle (rad)
kf.q_noise = deg2rad(0.1);      % pitch rate (rad/s)
kf.w_noise = 0.05;              % vertical velocity (m/s)

% Process noise (tuning parameter)
kf.q_process = 0.001 * eye(4);  % process noise covariance

%% Simulation Parameters

sim.dt = 0.01;                  % time step (100 Hz)
sim.t_final = 60;               % duration (s)

% Commands and disturbances
sim.theta_ref = deg2rad(10);    % pitch angle reference (rad)
sim.t_cmd = 5;                  % time to issue command (s)

% Wind gust
sim.w_gust_mag = 1;             % gust magnitude (m/s)
sim.t_gust_start = 15;          % gust start time (s)
sim.t_gust_end = 17;            % gust end time (s)

%% Save all parameters
save('parameters.mat', 'aircraft', 'lqr', 'kf', 'sim');
fprintf('Parameters saved to parameters.mat\n');
