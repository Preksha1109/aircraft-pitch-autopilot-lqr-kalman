%% Aircraft Pitch Autopilot - Longitudinal Dynamics Model
% This script defines the linearized state-space aircraft model
% for a typical single-engine aircraft (Cessna 172-like)
%
% State vector: x = [u, w, q, theta]^T
%   u       : forward velocity perturbation (m/s)
%   w       : vertical velocity perturbation (m/s)
%   q       : pitch rate (rad/s)
%   theta   : pitch angle (rad)
%
% Input: elevator deflection delta_e (rad)
% Output: pitch angle theta, pitch rate q, vertical velocity w

clear all; close all; clc;

%% Aircraft Parameters (Cessna 172-like, 6U configuration)
% Reference: Typical single-engine aircraft data

% Stability and control derivatives (non-dimensional, per radian)
% Speed: V0 = 30 m/s (cruise), altitude 500 m, standard atmosphere

V0 = 30;                    % cruise speed (m/s)
g = 9.81;                   % gravity (m/s^2)
m = 1100;                   % aircraft mass (kg)
Iyy = 1285;                 % pitch moment of inertia (kg·m^2)

% Aerodynamic derivatives at cruise (V0 = 30 m/s, rho = 1.22 kg/m^3)
% Data typically from wind-tunnel or flight test

Xu = -0.041;                % X_u/m : drag damping in u
Xw = 0.0;                   % X_w/m : unused (alpha effects on drag)
Xde = 0.0;                  % X_de/m : elevator has negligible drag effect

Zu = -0.25;                 % Z_u/m : vertical force derivative wrt u
Zw = -3.2;                  % Z_w/m : vertical force derivative wrt w (lift)
Zq = -0.15;                 % Z_q/m : pitch rate damping
Zde = -0.5;                 % Z_de/m : elevator control effectiveness

Mu = 0.011;                 % M_u/Iyy : pitch moment derivative wrt u
Mw = -0.012;                % M_w/Iyy : pitch moment derivative wrt w (pitch damping)
Mq = -0.45;                 % M_q/Iyy : pitch rate damping
Mde = 0.165;                % M_de/Iyy : elevator control effectiveness

% Trim condition: level flight at V0
theta0 = 0;                 % pitch angle at trim (rad)
alpha0 = 0.05;              % angle of attack at trim (rad ~ 3 deg)

%% Build state-space model: x' = Ax + Bu, y = Cx + Du
%  State order: [u, w, q, theta]

% State matrix A (4x4)
A = [
    Xu,     Xw,    0,      -g*cos(theta0);
    Zu,     Zw,    V0+Zq,  -g*sin(theta0);
    Mu,     Mw,    Mq,     0;
    0,      0,     1,      0
];

% Input matrix B (4x1) for elevator deflection
B = [
    Xde;
    Zde;
    Mde;
    0
];

% Output matrix C
% Typically measure pitch angle, pitch rate, and vertical velocity
C = [
    0 0 0 1;    % pitch angle theta
    0 0 1 0;    % pitch rate q
    0 1 0 0     % vertical velocity w
];

D = zeros(3, 1);

% Create state-space model
aircraft = ss(A, B, C, D);
aircraft.StateName = {'u (m/s)', 'w (m/s)', 'q (rad/s)', '\theta (rad)'};
aircraft.InputName = {'\delta_e (rad)'};
aircraft.OutputName = {'\theta (rad)', 'q (rad/s)', 'w (m/s)'};

%% Stability analysis
disp('=== Aircraft Longitudinal Dynamics Model ===');
disp('State vector: x = [u, w, q, theta]^T');
disp('Cruise speed: 30 m/s');
disp('');

% Eigenvalues of open-loop system
evals = eig(A);
fprintf('Open-loop eigenvalues:\n');
for i = 1:length(evals)
    if imag(evals(i)) == 0
        fprintf('  λ%d = %.4f (real)\n', i, real(evals(i)));
    else
        fprintf('  λ%d = %.4f ± j%.4f\n', i, real(evals(i)), abs(imag(evals(i))));
    end
end

% Short-period and phugoid modes
fprintf('\nMode identification:\n');
fprintf('  Stable real root ~ short-period oscillation\n');
fprintf('  Complex pair ~ phugoid (long-period) oscillation\n');

% Natural frequency and damping (for second-order component)
[wn, zeta, poles] = damp(A);
fprintf('\nNatural frequencies and damping:\n');
fprintf('  Mode 1: ωn = %.3f rad/s, ζ = %.3f\n', wn(1), zeta(1));
fprintf('  Mode 2: ωn = %.3f rad/s, ζ = %.3f\n', wn(2), zeta(2));
fprintf('  Mode 3: ωn = %.3f rad/s, ζ = %.3f\n', wn(3), zeta(3));
fprintf('  Mode 4: ωn = %.3f rad/s, ζ = %.3f\n', wn(4), zeta(4));

%% Save model to workspace and file
save('aircraft_model.mat', 'aircraft', 'A', 'B', 'C', 'D', 'V0', 'm', 'Iyy', 'g');
fprintf('\nModel saved to aircraft_model.mat\n');

%% Quick frequency response (Bode plot)
figure('Name', 'Open-Loop Bode Plot');
bode(aircraft);
grid on;
title('Aircraft Longitudinal Dynamics - Pitch Angle to Elevator Deflection');
legend('Pitch angle θ(s)/δ_e(s)', 'Pitch rate q(s)/δ_e(s)', 'Vertical vel w(s)/δ_e(s)');

savefig('figs/01_open_loop_bode.fig');
print('figs/01_open_loop_bode.png', '-dpng', '-r150');
close();

fprintf('\nFrequency response plots saved.\n');
