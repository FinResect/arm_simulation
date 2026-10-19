clear
clc
close all

source_dir = fileparts(mfilename('fullpath'));
addpath(fullfile(source_dir, 'model'));

%% User settings
% Cylinder centre in the scene frame. Fill each range as [min, max].
% Use [value, value] to hold a coordinate or angle fixed.
position_unit = 'mm'; % 'mm' or 'm'; converted to metres before calling TechCore.
x_range = [-100, 0];
y_range = [-200, 200];
z_range = [500, 700]; % Cylinder centre Z is 320 mm above the tool tip.

% Angles in degrees: theta [-90,90], phi [0,90], alpha [-45,45].
theta_range = [-90, 90];
phi_range = [0, 90];
alpha_range = [-45, 45];

% Uniform random positions, each with independently sampled orientations.
position_samples = 100;
orientations_per_position = 20;
random_seed = 0; % Change the seed for a different reproducible point cloud.
use_parallel = true;
progress_every = 100; % Report every N completed poses; 0 disables periodic reports.
print_each_sample = false; % true prints PASS/FAIL and errors for every completed pose.
show_solver_stages = true; % Show startup/IK stages until the first completion; all if print_each_sample.
parallel_workers = 4; % Conservative default; increase after successful pool startup.
base_height_range = [290, 490]; % Baseline height in mm; each pose may choose its own height.
ik_seed = 0; % Per-pose random IK starts are identical in serial/parallel modes.
R_cylinder_tool = eye(3); % Tool axes expressed in the cylinder frame.
ik_attempts = 3;
position_tolerance = 1e-4; % Metres.
orientation_tolerance = 1e-3; % Radians.
max_samples = 100000;

%% Validate settings before creating figures or running IK
range_values = {x_range, y_range, z_range, theta_range, phi_range, alpha_range};
range_names = {'x_range', 'y_range', 'z_range', 'theta_range', 'phi_range', 'alpha_range'};
missing = cellfun(@isempty, range_values);

if any(missing)
    fprintf('Fill these cylinder ranges as [min, max] before running:\n%s\n', ...
        strjoin(range_names(missing), ', '));
    return;
end

ranges = zeros(6, 2);

for k = 1:6
    validateattributes(range_values{k}, {'double'}, ...
        {'real', 'finite', 'vector', 'numel', 2}, mfilename, range_names{k});
    ranges(k, :) = reshape(range_values{k}, 1, 2);
end

switch position_unit
    case 'mm'
        ranges(1:3, :) = ranges(1:3, :) / 1000;
    case 'm'
    otherwise
        error('main:InvalidUnit', 'position_unit must be mm or m.');
end

base_height_range = base_height_range / 1000;

%% Evaluate all sampled cylinder poses without changing the arm configuration
rng(random_seed);
arm = ArmModel('Show', false);
core = TechCore('ToolRotation', R_cylinder_tool);
fprintf('Random sampling: %d positions, %d orientations per position (%d poses).\n', ...
    position_samples, orientations_per_position, position_samples * orientations_per_position);
fprintf('Position input unit: %s; solver and plot coordinates are in metres.\n', position_unit);
fprintf('Base lift range: %.0f-%.0f mm; each pose may use a different height.\n', ...
    base_height_range * 1000);
timer_value = tic;
result = core.evaluate(arm, ranges, [position_samples, orientations_per_position], ...
    'Sampling', 'random', ...
    'UseParallel', use_parallel, 'Workers', parallel_workers, 'IKSeed', ik_seed, ...
    'BaseHeightRange', base_height_range, ...
    'ProgressEvery', progress_every, 'PrintEachSample', print_each_sample, ...
    'ShowSolverStages', show_solver_stages, ...
    'Attempts', ik_attempts, ...
    'PositionTolerance', position_tolerance, ...
    'OrientationTolerance', orientation_tolerance, ...
    'MaxSamples', max_samples);

fprintf('Valid IK solutions: %d / %d sampled poses.\n', ...
    nnz(result.success), numel(result.success));
fprintf('Cylinder positions with all sampled orientations passing: %d / %d.\n', ...
    nnz(result.successFraction == 1), numel(result.successFraction));
fprintf('All sampled poses passed: %d. Elapsed time: %.2f s.\n', ...
    result.allSampledPosesPassed, toc(timer_value));
fprintf('Failed IK searches are not proof of unreachability; collision/path checks are not included.\n');
fprintf('Parallel setup: %.2f s. IK evaluation: %.2f s.\n', ...
    result.parallelSetupTime, result.solveTime);

if any(result.success)
    fprintf('Successful base heights min/median/max (mm): %.3f / %.3f / %.3f.\n', ...
        min(result.baseHeight(result.success)) * 1000, ...
        median(result.baseHeight(result.success)) * 1000, ...
        max(result.baseHeight(result.success)) * 1000);
end

%% Failure diagnostics (exit flag 5 alone does not indicate IK failure)
failed = ~result.success;

if any(failed)
    fprintf('Failed samples with position residual above tolerance: %d / %d.\n', ...
        nnz(failed & result.positionError > position_tolerance), nnz(failed));
    fprintf('Failed samples with orientation residual above tolerance: %d / %d.\n', ...
        nnz(failed & result.orientationError > orientation_tolerance), nnz(failed));
    fprintf('Failed samples without a positive finite solver exit flag: %d / %d.\n', ...
        nnz(failed & (~isfinite(result.exitFlag) | result.exitFlag <= 0)), nnz(failed));
    finite_residuals = failed & isfinite(result.positionError) & isfinite(result.orientationError);

    if any(finite_residuals)
        fprintf('Failed-sample position residual min/median/max (mm): %.6g / %.6g / %.6g.\n', ...
            min(result.positionError(finite_residuals)) * 1000, ...
            median(result.positionError(finite_residuals)) * 1000, ...
            max(result.positionError(finite_residuals)) * 1000);
        fprintf('Failed-sample orientation residual min/median/max (deg): %.6g / %.6g / %.6g.\n', ...
            rad2deg(min(result.orientationError(finite_residuals))), ...
            rad2deg(median(result.orientationError(finite_residuals))), ...
            rad2deg(max(result.orientationError(finite_residuals))));
    end

    failed_indices = find(failed);
    failed_diagnostics = table(failed_indices, result.exitFlag(failed), ...
        result.positionError(failed) * 1000, rad2deg(result.orientationError(failed)), ...
        result.attempts(failed), 'VariableNames', ...
        {'SampleIndex', 'ExitFlag', 'PositionError_mm', 'OrientationError_deg', 'Attempts'});
    fprintf('First failed samples (best residual attempt; counts above can overlap):\n');
    disp(failed_diagnostics(1:min(20, height(failed_diagnostics)), :));
end

%% Figure 1: cylinder centres, grouped by sampled orientation results
cylinder_axes = core.draw_cloud(result);
core.draw_reference(arm, cylinder_axes, base_height_range);

%% Figure 2: actual tool target points, including the -75 mm cylinder-X offset
target_figure = figure('Name', 'Tool Target Point Cloud', 'NumberTitle', 'off');
target_axes = axes(target_figure);
p = result.targetPositionsScene;
scatter3(target_axes, p(:, 1), p(:, 2), p(:, 3), ...
    20, double(result.success), 'filled');
colormap(target_axes, [0.8, 0.15, 0.15; 0.1, 0.65, 0.25]);
clim(target_axes, [-0.5, 1.5]);
target_colorbar = colorbar(target_axes);
target_colorbar.Ticks = [0, 1];
target_colorbar.TickLabels = {'No valid IK found', 'Valid IK found'};
grid(target_axes, 'on');
axis(target_axes, 'equal');
xlabel(target_axes, 'Scene X (m)');
ylabel(target_axes, 'Scene Y (m)');
zlabel(target_axes, 'Scene Z (m)');
title(target_axes, 'Tool target samples in the fixed scene frame');
core.draw_reference(arm, target_axes, base_height_range);
