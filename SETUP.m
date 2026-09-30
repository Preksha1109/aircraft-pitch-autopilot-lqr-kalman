%% Aircraft Pitch Autopilot - Setup Script
% Initialize MATLAB environment and verify dependencies

clc; clear all; close all;

fprintf('\n');
fprintf('╔══════════════════════════════════════════════════════════════╗\n');
fprintf('║           Aircraft Pitch Autopilot - Setup                  ║\n');
fprintf('╚══════════════════════════════════════════════════════════════╝\n\n');

%% Check MATLAB version
fprintf('Checking MATLAB version... ');
v = version;
fprintf('%s\n', v);

%% Check required toolboxes
fprintf('\nChecking required toolboxes...\n');

required_toolboxes = {
    'Control System Toolbox'
    'Optimization Toolbox'
};

for i = 1:length(required_toolboxes)
    toolbox = required_toolboxes{i};
    if ~isempty(which('lqr')) && ~isempty(which('lqe'))
        fprintf('  ✓ %s: available\n', toolbox);
    else
        fprintf('  ✗ %s: NOT FOUND\n', toolbox);
        fprintf('    Install via: Home > Add-Ons > Get Add-Ons\n');
    end
end

%% Setup paths
fprintf('\nSetting up MATLAB paths...\n');

% Add matlab directory to path
matlab_dir = fullfile(pwd, 'matlab');
if ~any(strcmp(path, matlab_dir))
    addpath(matlab_dir);
    fprintf('  ✓ Added %s to path\n', matlab_dir);
else
    fprintf('  ✓ Path already configured\n');
end

% Create output directory
figs_dir = fullfile(matlab_dir, 'figs');
if ~exist(figs_dir, 'dir')
    mkdir(figs_dir);
    fprintf('  ✓ Created output directory: %s\n', figs_dir);
else
    fprintf('  ✓ Output directory exists: %s\n', figs_dir);
end

%% Verify key files
fprintf('\nVerifying required files...\n');

required_files = {
    'matlab/aircraft_model.m'
    'matlab/lqr_controller.m'
    'matlab/kalman_filter.m'
    'matlab/simulate_autopilot.m'
    'matlab/plot_simulation_results.m'
    'matlab/parameters.m'
    'matlab/run_autopilot.m'
    'matlab/test_suite.m'
    'README.md'
};

all_present = true;
for i = 1:length(required_files)
    file = required_files{i};
    if isfile(file)
        fprintf('  ✓ %s\n', file);
    else
        fprintf('  ✗ %s NOT FOUND\n', file);
        all_present = false;
    end
end

%% Summary
fprintf('\n');
fprintf('╔══════════════════════════════════════════════════════════════╗\n');
fprintf('║                    Setup Complete                           ║\n');
fprintf('╚══════════════════════════════════════════════════════════════╝\n\n');

if all_present
    fprintf('✓ All files found. Ready to run!\n\n');
    fprintf('Next steps:\n');
    fprintf('  1. Review parameters in matlab/parameters.m (optional)\n');
    fprintf('  2. Run: run_autopilot\n');
    fprintf('  3. View results in figs/ directory\n');
    fprintf('  4. For sensitivity analysis, run: analyze_robustness\n\n');
else
    fprintf('✗ Some files missing. Check installation.\n\n');
end

fprintf('For detailed help, see README.md\n\n');
