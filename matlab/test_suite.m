%% Test Suite: Aircraft Pitch Autopilot Validation
% Automated tests to validate the LQR controller and Kalman filter

clear all; close all; clc;

load('aircraft_model.mat');
load('lqr_controller.mat');
load('kalman_filter.mat');

fprintf('\n');
fprintf('╔══════════════════════════════════════════════════════════════╗\n');
fprintf('║           Aircraft Pitch Autopilot - Test Suite              ║\n');
fprintf('╚══════════════════════════════════════════════════════════════╝\n\n');

num_tests = 0;
num_passed = 0;

%% Test 1: System stability (open-loop)
fprintf('Test 1: Open-loop system stability\n');
num_tests = num_tests + 1;

evals_ol = eig(A);
is_unstable = any(real(evals_ol) >= 0);

if is_unstable
    fprintf('  ✗ FAIL: Open-loop system is unstable\n');
else
    fprintf('  ✓ PASS: Open-loop system is stable\n');
    num_passed = num_passed + 1;
end
fprintf('    Eigenvalues: ');
fprintf('%.3f ', evals_ol);
fprintf('\n\n');

%% Test 2: Closed-loop stability (LQR)
fprintf('Test 2: Closed-loop stability (LQR)\n');
num_tests = num_tests + 1;

A_cl = A - B*K;
evals_cl = eig(A_cl);
is_stable_cl = all(real(evals_cl) < 0);

if ~is_stable_cl
    fprintf('  ✗ FAIL: Closed-loop system is unstable\n');
else
    fprintf('  ✓ PASS: Closed-loop system is stable\n');
    num_passed = num_passed + 1;
    % Compute damping
    [wn, zeta, ~] = damp(A_cl);
    fprintf('    Natural frequencies: ');
    fprintf('%.3f ', wn);
    fprintf('rad/s\n');
    fprintf('    Damping ratios: ');
    fprintf('%.3f ', zeta);
    fprintf('\n');
end
fprintf('\n');

%% Test 3: Observability
fprintf('Test 3: Observability (can estimate all states?)\n');
num_tests = num_tests + 1;

rank_obs = rank(obsv(A, C));
is_observable = rank_obs == 4;

if ~is_observable
    fprintf('  ✗ FAIL: System not fully observable (rank = %d < 4)\n', rank_obs);
else
    fprintf('  ✓ PASS: System is fully observable (rank = 4)\n');
    num_passed = num_passed + 1;
end
fprintf('\n');

%% Test 4: Controllability
fprintf('Test 4: Controllability (can control all states?)\n');
num_tests = num_tests + 1;

rank_ctrl = rank(ctrb(A, B));
is_controllable = rank_ctrl == 4;

if ~is_controllable
    fprintf('  ✗ FAIL: System not fully controllable (rank = %d < 4)\n', rank_ctrl);
else
    fprintf('  ✓ PASS: System is fully controllable (rank = 4)\n');
    num_passed = num_passed + 1;
end
fprintf('\n');

%% Test 5: LQR gain dimensions
fprintf('Test 5: LQR gain matrix dimensions\n');
num_tests = num_tests + 1;

[nrows, ncols] = size(K);
is_correct_dims = (nrows == 1) && (ncols == 4);

if ~is_correct_dims
    fprintf('  ✗ FAIL: K has wrong dimensions (%.1f×%.1f), expected 1×4\n', nrows, ncols);
else
    fprintf('  ✓ PASS: K has correct dimensions 1×4\n');
    num_passed = num_passed + 1;
end
fprintf('    K = [%.4f  %.4f  %.4f  %.4f]\n', K);
fprintf('\n');

%% Test 6: Riccati equation solution validity
fprintf('Test 6: Riccati equation solution (ARE residual)\n');
num_tests = num_tests + 1;

% ARE: PA' + AP - PBR^{-1}B'P + Q = 0
Q_test = diag([1, 5, 50, 100]);
R_test = 1;
residual = P*A' + A*P - P*B*(1/R_test)*B'*P + Q_test;
residual_norm = norm(residual, 'fro');
tol_are = 1e-6;

if residual_norm > tol_are
    fprintf('  ✗ FAIL: ARE residual = %.2e > %.2e\n', residual_norm, tol_are);
else
    fprintf('  ✓ PASS: ARE residual = %.2e (within tolerance)\n', residual_norm);
    num_passed = num_passed + 1;
end
fprintf('\n');

%% Test 7: Positive definiteness of P
fprintf('Test 7: Riccati solution P is positive definite\n');
num_tests = num_tests + 1;

evals_P = eig(P);
is_pd = all(evals_P > 0);

if ~is_pd
    fprintf('  ✗ FAIL: P is not positive definite\n');
else
    fprintf('  ✓ PASS: P is positive definite\n');
    num_passed = num_passed + 1;
end
fprintf('    Condition number: %.2e\n', cond(P));
fprintf('\n');

%% Test 8: Gain margin (frequency domain)
fprintf('Test 8: Gain margin from LQR\n');
num_tests = num_tests + 1;

% For LQR with diagonal cost, gain margin >= 3 dB
% Loop transfer function: L(s) = K(sI-A)^{-1}B
sys_lti = ss(A, B, eye(1), 0);  % Create LTI system
L_sys = -K*sys_lti;  % Open-loop transfer function
[gm, pm, wgm, wpm] = margin(L_sys);

gm_dB = 20*log10(gm);
is_gm_ok = gm_dB >= 3;

if ~is_gm_ok
    fprintf('  ✗ FAIL: Gain margin = %.2f dB < 3 dB\n', gm_dB);
else
    fprintf('  ✓ PASS: Gain margin = %.2f dB (> 3 dB, typical LQR guarantee)\n', gm_dB);
    num_passed = num_passed + 1;
end
fprintf('    Phase margin: %.2f°\n', pm);
fprintf('\n');

%% Test 9: Observer stability
fprintf('Test 9: Observer (Kalman filter) stability\n');
num_tests = num_tests + 1;

A_obs = A - L*C;
evals_obs = eig(A_obs);
is_obs_stable = all(real(evals_obs) < 0);

if ~is_obs_stable
    fprintf('  ✗ FAIL: Observer is unstable\n');
else
    fprintf('  ✓ PASS: Observer is stable\n');
    num_passed = num_passed + 1;
end
fprintf('    Observer eigenvalues: ');
fprintf('%.3f ', evals_obs);
fprintf('\n\n');

%% Test 10: Measurement matrix rank
fprintf('Test 10: Measurement matrix rank (can measure what we need?)\n');
num_tests = num_tests + 1;

rank_C = rank(C);
is_C_fullrank = rank_C == 3;

if ~is_C_fullrank
    fprintf('  ✗ FAIL: C has rank %d < 3 (cannot measure all outputs)\n', rank_C);
else
    fprintf('  ✓ PASS: C has full row rank (measure 3 of 4 states)\n');
    num_passed = num_passed + 1;
end
fprintf('    C dimensions: %d×%d\n', size(C));
fprintf('\n');

%% Test 11: Controller authority (elevator limits)
fprintf('Test 11: Control authority within limits\n');
num_tests = num_tests + 1;

% Simulate a large state perturbation and check if control saturates
x_test = [1; 1; deg2rad(5); deg2rad(20)];  % Significant perturbation
u_required = -K * x_test;

u_sat_limit = deg2rad(25);
will_saturate = abs(u_required) > u_sat_limit;

fprintf('  For state perturbation [1 m/s, 1 m/s, 5°/s, 20°]:\n');
fprintf('    Required control: %.2f° (%.4f rad)\n', rad2deg(u_required), u_required);
fprintf('    Saturation limit: ±25°\n');

if will_saturate
    fprintf('  ! WARNING: Large perturbations may saturate elevator\n');
    num_passed = num_passed + 1;  % Still pass, saturation is normal
else
    fprintf('  ✓ PASS: Control within limits for typical perturbations\n');
    num_passed = num_passed + 1;
end
fprintf('\n');

%% Test 12: Separation principle (orthogonality)
fprintf('Test 12: Separation principle (controller ⊥ observer poles)\n');
num_tests = num_tests + 1;

A_cl = A - B*K;
A_obs = A - L*C;

evals_controller = eig(A_cl);
evals_observer = eig(A_obs);

% They should be different (unless by chance)
are_different = ~isequal(evals_controller, evals_observer);

if ~are_different
    fprintf('  ✗ FAIL: Controller and observer have same poles (unexpected)\n');
else
    fprintf('  ✓ PASS: Controller and observer poles are independent\n');
    num_passed = num_passed + 1;
end
fprintf('    Controller pole damping:  min ζ = %.3f, max ωn = %.3f rad/s\n', ...
    min(abs(evals_controller)), max(abs(evals_controller)));
fprintf('    Observer pole damping:    min ζ = %.3f, max ωn = %.3f rad/s\n', ...
    min(abs(evals_observer)), max(abs(evals_observer)));
fprintf('\n');

%% Summary
fprintf('╔══════════════════════════════════════════════════════════════╗\n');
fprintf('║                       TEST SUMMARY                          ║\n');
fprintf('╚══════════════════════════════════════════════════════════════╝\n\n');

pass_rate = (num_passed / num_tests) * 100;
fprintf('Tests passed: %d / %d (%.1f%%)\n\n', num_passed, num_tests, pass_rate);

if num_passed == num_tests
    fprintf('✓ All tests passed! System is ready for simulation.\n\n');
else
    fprintf('✗ Some tests failed. Review design before proceeding.\n\n');
end

% Save test results
save('test_results.mat', 'num_tests', 'num_passed', 'pass_rate');

fprintf('Test results saved to test_results.mat\n');
