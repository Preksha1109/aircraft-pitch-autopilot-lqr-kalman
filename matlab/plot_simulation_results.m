%% Plot Simulation Results
% Comprehensive visualization of closed-loop autopilot performance

load('simulation_results.mat');

fprintf('Generating plots...\n');

%% Figure 1: Pitch angle tracking
figure('Name', 'Pitch Angle Tracking', 'Position', [100 100 900 600]);

plot(t, theta_true, 'b--', 'LineWidth', 1.5, 'DisplayName', 'True');
hold on;
plot(t, theta_est, 'g-', 'LineWidth', 1.5, 'DisplayName', 'Estimated');
plot(t(1:length(t)), 10*ones(size(t)), 'r--', 'LineWidth', 1, 'DisplayName', 'Reference (10°)');
hold off;

grid on;
xlabel('Time (s)', 'FontSize', 11);
ylabel('Pitch Angle (deg)', 'FontSize', 11);
title('Pitch Angle: True vs Estimated (Kalman Filter)', 'FontSize', 12, 'FontWeight', 'bold');
legend('FontSize', 10);
xlim([0 t(end)]);

savefig('figs/06_pitch_angle_tracking.fig');
print('figs/06_pitch_angle_tracking.png', '-dpng', '-r150');
close();

%% Figure 2: Estimation error
figure('Name', 'Estimation Error', 'Position', [100 100 900 600]);

subplot(2,2,1);
plot(t, theta_error*rad2deg(1), 'b', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)'); ylabel('Error (deg)');
title('Pitch Angle Estimation Error');

subplot(2,2,2);
plot(t, x_error(2,:), 'r', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)'); ylabel('Error (m/s)');
title('Vertical Velocity Estimation Error');

subplot(2,2,3);
plot(t, x_error(3,:)*rad2deg(1), 'g', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)'); ylabel('Error (deg/s)');
title('Pitch Rate Estimation Error');

subplot(2,2,4);
plot(t, x_error(1,:), 'm', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)'); ylabel('Error (m/s)');
title('Forward Velocity Estimation Error');

sgtitle('Kalman Filter Estimation Errors', 'FontSize', 12, 'FontWeight', 'bold');

savefig('figs/07_estimation_error.fig');
print('figs/07_estimation_error.png', '-dpng', '-r150');
close();

%% Figure 3: State evolution
figure('Name', 'State Evolution', 'Position', [100 100 900 700]);

% True state
subplot(2,2,1);
plot(t, x_true(1,:), 'b', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)'); ylabel('Velocity (m/s)');
title('Forward Velocity u');

subplot(2,2,2);
plot(t, x_true(2,:), 'r', 'LineWidth', 1.5);
hold on;
idx_gust = find(t >= 15 & t <= 17);
patch([t(idx_gust(1)) t(idx_gust(end)) t(idx_gust(end)) t(idx_gust(1))], ...
      [-5 -5 5 5], 'yellow', 'FaceAlpha', 0.2, 'EdgeColor', 'none');
hold off;
grid on;
xlabel('Time (s)'); ylabel('Velocity (m/s)');
title('Vertical Velocity w (with wind gust region)');

subplot(2,2,3);
plot(t, x_true(3,:)*rad2deg(1), 'g', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)'); ylabel('Rate (deg/s)');
title('Pitch Rate q');

subplot(2,2,4);
plot(t, x_true(4,:)*rad2deg(1), 'k', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)'); ylabel('Angle (deg)');
title('Pitch Angle θ');

sgtitle('Aircraft True State', 'FontSize', 12, 'FontWeight', 'bold');

savefig('figs/08_state_evolution.fig');
print('figs/08_state_evolution.png', '-dpng', '-r150');
close();

%% Figure 4: Control input
figure('Name', 'Control Input', 'Position', [100 100 900 500]);

subplot(2,1,1);
plot(t, u_cmd*rad2deg(1), 'b--', 'LineWidth', 1, 'DisplayName', 'Unsaturated command');
hold on;
plot(t, u_sat*rad2deg(1), 'r-', 'LineWidth', 1.5, 'DisplayName', 'Saturated (applied)');
plot(t, 25*ones(size(t)), 'k--', 'LineWidth', 1, 'DisplayName', 'Saturation limit (±25°)');
plot(t, -25*ones(size(t)), 'k--', 'LineWidth', 1);
hold off;
grid on;
ylabel('Elevator Deflection (deg)', 'FontSize', 11);
title('Elevator Command with Saturation', 'FontSize', 12, 'FontWeight', 'bold');
legend('FontSize', 10, 'Location', 'southeast');
xlim([0 t(end)]);
ylim([-30 30]);

subplot(2,1,2);
% Zoom in on transient
idx_zoom = find(t >= 4.5 & t <= 15);
plot(t(idx_zoom), u_sat(idx_zoom)*rad2deg(1), 'r-', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)', 'FontSize', 11);
ylabel('Elevator Deflection (deg)', 'FontSize', 11);
title('Elevator Transient Response (zoomed)', 'FontSize', 11);
xlim([t(idx_zoom(1)) t(idx_zoom(end))]);

savefig('figs/09_control_input.fig');
print('figs/09_control_input.png', '-dpng', '-r150');
close();

%% Figure 5: Sensor measurements
figure('Name', 'Sensor Measurements', 'Position', [100 100 900 700]);

subplot(3,1,1);
plot(t, z_meas(1,:)*rad2deg(1), 'b.', 'MarkerSize', 3, 'DisplayName', 'Noisy measurement');
hold on;
plot(t, theta_true, 'g-', 'LineWidth', 1.5, 'DisplayName', 'True (noiseless)');
hold off;
grid on;
ylabel('Angle (deg)');
title('Pitch Angle Measurement (with sensor noise)', 'FontSize', 11);
legend('FontSize', 9);

subplot(3,1,2);
plot(t, z_meas(2,:)*rad2deg(1), 'r.', 'MarkerSize', 3, 'DisplayName', 'Noisy measurement');
hold on;
plot(t, x_true(3,:)*rad2deg(1), 'g-', 'LineWidth', 1.5, 'DisplayName', 'True (noiseless)');
hold off;
grid on;
ylabel('Rate (deg/s)');
title('Pitch Rate Measurement (with sensor noise)', 'FontSize', 11);
legend('FontSize', 9);

subplot(3,1,3);
plot(t, z_meas(3,:), 'b.', 'MarkerSize', 3, 'DisplayName', 'Noisy measurement');
hold on;
plot(t, x_true(2,:), 'g-', 'LineWidth', 1.5, 'DisplayName', 'True (noiseless)');
hold off;
grid on;
xlabel('Time (s)');
ylabel('Velocity (m/s)');
title('Vertical Velocity Measurement (with sensor noise)', 'FontSize', 11);
legend('FontSize', 9);

sgtitle('Sensor Measurements vs True State', 'FontSize', 12, 'FontWeight', 'bold');

savefig('figs/10_sensor_measurements.fig');
print('figs/10_sensor_measurements.png', '-dpng', '-r150');
close();

%% Figure 6: Performance summary
figure('Name', 'Performance Summary', 'Position', [100 100 900 600]);

% Pitch angle with command and reference band
plot(t, theta_true, 'b--', 'LineWidth', 1, 'DisplayName', 'True', 'Color', [0.3 0.5 1.0]);
hold on;
plot(t, theta_est, 'g-', 'LineWidth', 2, 'DisplayName', 'Estimated (Kalman filter)');
plot(t, 10*ones(size(t)), 'r--', 'LineWidth', 1.5, 'DisplayName', 'Reference');
% 5% band
patch([t, fliplr(t)], [10.5*ones(size(t)), 9.5*ones(size(t))], ...
      'green', 'FaceAlpha', 0.1, 'EdgeColor', 'none');
text(t(end)*0.5, 10.7, '±5% band', 'FontSize', 9, 'Color', 'green');
hold off;

grid on;
xlabel('Time (s)', 'FontSize', 11);
ylabel('Pitch Angle (deg)', 'FontSize', 11);
title('Aircraft Pitch Autopilot Performance Summary', 'FontSize', 12, 'FontWeight', 'bold');
legend('FontSize', 10);
xlim([0 t(end)]);
ylim([0 12]);

% Add annotation
txt = sprintf('Step command: 10° at t=5s\nWind gust: 1 m/s at t=15-17s');
annotation('textbox', [0.15 0.6 0.25 0.3], 'String', txt, 'FontSize', 9, ...
    'BackgroundColor', 'white', 'EdgeColor', 'black');

savefig('figs/11_performance_summary.fig');
print('figs/11_performance_summary.png', '-dpng', '-r150');
close();

fprintf('All plots saved to figs/ directory.\n');
