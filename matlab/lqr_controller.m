%% LQR Controller Design for Aircraft Pitch Autopilot
% Solves the Linear Quadratic Regulator problem:
% min J = ∫ (x'Qx + u'Ru) dt
%
% Equivalent to finding the optimal feedback gain K that minimizes
% the cost and stabilizes the system: u = -Kx

clear all; close all; clc;

% Load aircraft model
load('aircraft_model.mat');

fprintf('=== LQR Controller Design ===\n\n');

%% Define cost matrices Q and R
% Q weights state tracking error, R weights control effort
% Higher Q → tighter tracking but more control activity
% Higher R → smoother control but slower response

% Strategy: tune Q to prioritize pitch angle control
% w = [u, w, q, theta]
%
% We care most about theta (pitch angle), then q (pitch rate)
% Less emphasis on velocity perturbations u, w

% Tuning parameters (adjust these to tune response)
q_theta = 100;    % weight on pitch angle error (critical)
q_q = 50;         % weight on pitch rate (damping)
q_w = 5;          % weight on vertical velocity (moderate)
q_u = 1;          % weight on forward velocity (low)
r = 1;            % control effort weight

Q = diag([q_u, q_w, q_q, q_theta]);
R = r;

fprintf('Cost matrix Q (diagonal):\n');
disp(Q);
fprintf('Cost scalar R: %.2f\n\n', R);

%% Solve the Algebraic Riccati Equation
% dlqr(A,B,Q,R) returns optimal gain K and Riccati solution P
[K, P, evals_cl] = lqr(A, B, Q, R);

fprintf('Optimal state-feedback gain K:\n');
fprintf('  K = [%.4f  %.4f  %.4f  %.4f]\n', K);
fprintf('\nInterpretation:\n');
fprintf('  δ_e = -K₁*u - K₂*w - K₃*q - K₄*θ\n');
fprintf('  δ_e = -%.4f*u - %.4f*w - %.4f*q - %.4f*θ\n\n', K);

% Riccati solution P (Lyapunov matrix)
fprintf('Riccati solution P:\n');
disp(P);

%% Closed-loop system
A_cl = A - B*K;
sys_cl = ss(A_cl, B, C, D);
sys_cl.StateName = aircraft.StateName;
sys_cl.OutputName = aircraft.OutputName;

% Closed-loop eigenvalues
evals_cl = eig(A_cl);
fprintf('\n=== Closed-loop Eigenvalues ===\n');
fprintf('Open-loop vs. Closed-loop:\n');
evals_ol = eig(A);
for i = 1:length(evals_ol)
    fprintf('  OL: %.4f  →  CL: %.4f\n', evals_ol(i), evals_cl(i));
end

% Closed-loop damping and natural frequency
[wn_cl, zeta_cl, ~] = damp(A_cl);
fprintf('\nClosed-loop damping and natural frequency:\n');
for i = 1:length(wn_cl)
    fprintf('  Mode %d: ωn = %.3f rad/s, ζ = %.3f\n', i, wn_cl(i), zeta_cl(i));
end

% Settling time estimate (5% criterion)
ts_est = max(3./(zeta_cl.*wn_cl));
fprintf('\nEstimated settling time (5%% criterion): %.2f sec\n', ts_est);

%% Control input saturation
% Typical elevator deflection limits
delta_e_max = deg2rad(25);  % ±25 degrees
fprintf('\nControl input limits: ±%.2f deg (±%.4f rad)\n', rad2deg(delta_e_max), delta_e_max);

%% Frequency response - closed-loop system
figure('Name', 'Closed-Loop Bode Plot');
bode(sys_cl);
grid on;
title('Closed-Loop System - LQR Controlled Aircraft');
legend('Pitch angle', 'Pitch rate', 'Vertical velocity');

savefig('figs/02_closed_loop_bode.fig');
print('figs/02_closed_loop_bode.png', '-dpng', '-r150');
close();

%% Step response (pitch angle command)
figure('Name', 'Step Response');
[y_step, t_step] = step(sys_cl, 30);  % 30 second response

subplot(3,1,1);
plot(t_step, y_step(:,1)*rad2deg(1), 'b', 'LineWidth', 1.5);
grid on; ylabel('θ (deg)');
title('LQR Step Response: 1° Pitch Angle Step Command');
legend('Pitch angle');

subplot(3,1,2);
plot(t_step, y_step(:,2)*rad2deg(1), 'g', 'LineWidth', 1.5);
grid on; ylabel('q (deg/s)');
legend('Pitch rate');

subplot(3,1,3);
plot(t_step, y_step(:,3), 'r', 'LineWidth', 1.5);
grid on; ylabel('w (m/s)'); xlabel('Time (s)');
legend('Vertical velocity');

savefig('figs/03_step_response.fig');
print('figs/03_step_response.png', '-dpng', '-r150');
close();

%% Eigenvalue plot
figure('Name', 'Pole-Zero Map');
hold on;
plot(real(evals_ol), imag(evals_ol), 'ro', 'MarkerSize', 10, 'DisplayName', 'Open-loop poles');
plot(real(evals_cl), imag(evals_cl), 'b*', 'MarkerSize', 15, 'DisplayName', 'Closed-loop poles (LQR)');
grid on;
xlabel('Real axis (1/s)');
ylabel('Imaginary axis (rad/s)');
title('Pole Migration: Open-Loop → LQR Controlled');
legend;
axis equal;
xlim([-2 0.5]); ylim([-2 2]);

savefig('figs/04_pole_zero_map.fig');
print('figs/04_pole_zero_map.png', '-dpng', '-r150');
close();

%% Save LQR controller
save('lqr_controller.mat', 'K', 'P', 'Q', 'R', 'A_cl', 'sys_cl', 'delta_e_max');
fprintf('\n\nLQR controller saved to lqr_controller.mat\n');
