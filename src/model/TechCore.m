classdef TechCore < handle
    % Cylinder poses: metres and degrees; theta is azimuth, phi is polar angle.
    % alpha follows the right-hand rule about the outward cylinder X axis.
    properties (Constant)
        % p_base = T_base_scene * p_scene (homogeneous column vectors).
        T_base_scene = [-1, 0, 0, 0.1; 0, -1, 0, 0; 0, 0, 1, 0; 0, 0, 0, 1]
        p_cylinder_tool = [-0.075; 0; 0]
    end

    properties (SetAccess = private)
        R_cylinder_tool = eye(3)
    end

    methods
        function obj = TechCore(options)
            arguments
                options.ToolRotation (3, 3) double = eye(3)
            end
            obj.set_tool_rotation(options.ToolRotation);
        end

        function set_tool_rotation(obj, rotation)
            arguments
                obj
                rotation (3, 3) double {mustBeReal, mustBeFinite}
            end
            if norm(rotation.' * rotation - eye(3), 'fro') > 1e-8 || ...
                    abs(det(rotation) - 1) > 1e-8
                error('TechCore:InvalidRotation', 'ToolRotation must be a proper rotation matrix.');
            end
            obj.R_cylinder_tool = rotation;
        end

        function [T_base_tool, T_scene_cylinder, T_base_cylinder] = pose(obj, xyz, theta, phi, alpha)
            arguments
                obj
                xyz (1, 3) double {mustBeReal, mustBeFinite}
                theta (1, 1) double {mustBeReal, mustBeFinite, mustBeGreaterThanOrEqual(theta, -90), mustBeLessThanOrEqual(theta, 90)}
                phi (1, 1) double {mustBeReal, mustBeFinite, mustBeGreaterThanOrEqual(phi, 0), mustBeLessThanOrEqual(phi, 90)}
                alpha (1, 1) double {mustBeReal, mustBeFinite, mustBeGreaterThanOrEqual(alpha, -45), mustBeLessThanOrEqual(alpha, 45)}
            end
            Rz = [cosd(theta), -sind(theta), 0; sind(theta), cosd(theta), 0; 0, 0, 1];
            beta = phi - 90;
            Ry = [cosd(beta), 0, sind(beta); 0, 1, 0; -sind(beta), 0, cosd(beta)];
            Rx = [1, 0, 0; 0, cosd(alpha), -sind(alpha); 0, sind(alpha), cosd(alpha)];
            % This also defines the cylinder Y/Z axes at alpha = 0, including the pole.
            T_scene_cylinder = [Rz * Ry * Rx, xyz.'; 0, 0, 0, 1];
            T_base_cylinder = obj.T_base_scene * T_scene_cylinder;
            T_cylinder_tool = [obj.R_cylinder_tool, obj.p_cylinder_tool; 0, 0, 0, 1];
            T_base_tool = T_base_cylinder * T_cylinder_tool;
        end

        function cloud = sample_range(obj, ranges, counts, options)
            % Rows: x, y, z, theta, phi, alpha (metres/degrees).
            % Grid counts: six axis counts. Random counts: [positions, orientations].
            arguments
                obj
                ranges (6, 2) double {mustBeReal, mustBeFinite}
                counts (1, :) double {mustBeReal, mustBeFinite, mustBeInteger, mustBePositive}
                options.Sampling (1, 1) string {mustBeMember(options.Sampling, ["grid", "random"])} = "grid"
                options.MaxSamples (1, 1) double {mustBeReal, mustBeFinite, mustBeInteger, mustBePositive} = 100000
            end
            if any(ranges(:, 1) > ranges(:, 2)) || ...
                    any(ranges(4:6, 1) < [-90; 0; -45]) || ...
                    any(ranges(4:6, 2) > [90; 90; 45])
                error('TechCore:InvalidRanges', 'Ranges must be ordered and angles within their allowed bounds.');
            end
            if options.Sampling == "grid"
                if numel(counts) ~= 6
                    error('TechCore:InvalidCounts', 'Grid sampling needs six axis counts.');
                end
                fixed = ranges(:, 1) == ranges(:, 2);
                counts(fixed.') = 1;
                if any(counts(~fixed.') < 2)
                    error('TechCore:InvalidCounts', 'Nonconstant ranges need at least two samples.');
                end
            elseif numel(counts) ~= 2
                error('TechCore:InvalidCounts', 'Random sampling needs [position count, orientations per position].');
            end
            n = prod(counts);
            if n > options.MaxSamples
                error('TechCore:TooManySamples', 'Requested %g samples exceeds MaxSamples (%g).', n, options.MaxSamples);
            end
            if options.Sampling == "random"
                positions = ranges(1:3, 1).' + rand(counts(1), 3) .* diff(ranges(1:3, :), 1, 2).';
                angles = ranges(4:6, 1).' + rand(n, 3) .* diff(ranges(4:6, :), 1, 2).';
                % Uniform solid angle requires uniform cos(phi), not uniform phi.
                cosine = cosd(ranges(5, 2)) + rand(n, 1) * ...
                    (cosd(ranges(5, 1)) - cosd(ranges(5, 2)));
                angles(:, 2) = acosd(max(-1, min(1, cosine)));
                if ranges(5, 1) == ranges(5, 2)
                    angles(:, 2) = ranges(5, 1);
                end
                samples = [repelem(positions, counts(2), 1), angles];
            else
                values = cell(1, 6);
                grid = cell(1, 6);
                for k = 1:6
                    values{k} = linspace(ranges(k, 1), ranges(k, 2), counts(k));
                end
                [grid{:}] = ndgrid(values{:});
                samples = zeros(n, 6);
                for k = 1:6
                    samples(:, k) = grid{k}(:);
                end
            end
            transforms = zeros(4, 4, n);
            targetPositions = zeros(n, 3);
            for k = 1:n
                transforms(:, :, k) = obj.pose(samples(k, 1:3), samples(k, 4), samples(k, 5), samples(k, 6));
                targetPositions(k, :) = transforms(1:3, 4, k).';
            end
            cloud = struct('samples', samples, 'targetTransforms', transforms, ...
                'targetPositionsBase', targetPositions, 'counts', counts, 'sampling', options.Sampling);
        end

        function result = evaluate(obj, arm, ranges, counts, options)
            % Numerical IK evidence only; no collision or path-connectivity test.
            arguments
                obj
                arm ArmModel
                ranges (6, 2) double {mustBeReal, mustBeFinite}
                counts (1, :) double {mustBeReal, mustBeFinite, mustBeInteger, mustBePositive}
                options.Sampling (1, 1) string {mustBeMember(options.Sampling, ["grid", "random"])} = "grid"
                options.MaxSamples (1, 1) double {mustBeReal, mustBeFinite, mustBeInteger, mustBePositive} = 100000
                options.Attempts (1, 1) double {mustBeReal, mustBeFinite, mustBeInteger, mustBePositive} = 3
                options.PositionTolerance (1, 1) double {mustBeReal, mustBeFinite, mustBePositive} = 1e-4
                options.OrientationTolerance (1, 1) double {mustBeReal, mustBeFinite, mustBePositive} = 1e-3
                options.UseParallel (1, 1) logical = false
                options.Workers (1, 1) double {mustBeReal, mustBeFinite, mustBeInteger, mustBePositive} = 2
                options.ProgressEvery (1, 1) double {mustBeReal, mustBeFinite, mustBeInteger, mustBeNonnegative} = 100
                options.PrintEachSample (1, 1) logical = false
                options.ShowSolverStages (1, 1) logical = true
                options.BaseHeightRange (1, 2) double {mustBeReal, mustBeFinite, mustBeNonnegative} = [0.290, 0.490]
                options.IKSeed (1, 1) double {mustBeReal, mustBeFinite, mustBeInteger, mustBeNonnegative, mustBeLessThanOrEqual(options.IKSeed, 4294967295)} = 0
            end
            if options.BaseHeightRange(1) > options.BaseHeightRange(2)
                error('TechCore:InvalidHeightRange', 'Base height bounds must be ordered.');
            end
            result = obj.sample_range(ranges, counts, ...
                'Sampling', options.Sampling, 'MaxSamples', options.MaxSamples);
            n = size(result.samples, 1);
            success = false(n, 1);
            configurations = nan(n, arm.get_joint_count());
            positionError = inf(n, 1);
            orientationError = inf(n, 1);
            exitFlag = nan(n, 1);
            attempts = zeros(n, 1);
            baseHeights = nan(n, 1);
            useParallel = options.UseParallel;
            setupTimer = tic;
            workerLimit = 0;
            workerArm = [];
            result.parallelFallbackReason = '';
            if useParallel && (isempty(ver('parallel')) || ~license('test', 'Distrib_Computing_Toolbox'))
                warning('TechCore:ParallelUnavailable', ...
                    'Parallel Computing Toolbox is unavailable; using serial evaluation.');
                useParallel = false;
                result.parallelFallbackReason = 'Parallel Computing Toolbox is unavailable.';
            end
            if useParallel
                pool = gcp('nocreate');
                if isempty(pool)
                    fprintf('Starting a process pool with %d workers...\n', options.Workers);
                    try
                        pool = parpool('Processes', options.Workers);
                    catch exception
                        result.parallelFallbackReason = exception.getReport('basic', 'hyperlinks', 'off');
                        warning('TechCore:ParallelStartupFailed', ...
                            'Process pool startup failed; using serial evaluation.\n%s', ...
                            result.parallelFallbackReason);
                        useParallel = false;
                    end
                elseif isa(pool, 'parallel.ThreadPool')
                    error('TechCore:ProcessPoolRequired', ...
                        'Use a process pool for Robotics Toolbox IK; a thread pool is already open.');
                end
            end
            if useParallel
                workerLimit = min(options.Workers, pool.NumWorkers);
                % Each process receives an independent copy, including the current joint state.
                workerArm = parallel.pool.Constant(arm);
                cleanup = onCleanup(@() delete(workerArm));
                fprintf('Parallel IK evaluation: up to %d workers.\n', workerLimit);
            else
                fprintf('Serial IK evaluation.\n');
            end
            result.parallelSetupTime = toc(setupTimer);
            % Targets remain in the fixed scene; base height is an optimization variable.
            transforms = pagemtimes(inv(obj.T_base_scene), result.targetTransforms);
            result.targetTransformsScene = transforms;
            result.targetPositionsScene = reshape(transforms(1:3, 4, :), 3, []).';
            ikSeed = options.IKSeed;
            ikAttempts = options.Attempts;
            positionTolerance = options.PositionTolerance;
            orientationTolerance = options.OrientationTolerance;
            heightRange = options.BaseHeightRange;
            solveTimer = tic;
            completedCount = 0;
            startedCount = 0;
            passedCount = 0;
            progressEvery = options.ProgressEvery;
            printEachSample = options.PrintEachSample;
            showSolverStages = options.ShowSolverStages;
            reportEnabled = progressEvery > 0 || printEachSample || showSolverStages;
            progressCallback = @reportProgress;
            progressQueue = [];
            if useParallel
                progressQueue = parallel.pool.PollableDataQueue;
            end
            if useParallel
                futures = parallel.FevalFuture.empty;
                for batch = 1:min(workerLimit, n)
                    indices = batch:workerLimit:n;
                    futures(batch) = parfeval(pool, @solveLiftBatch, 1, workerArm, ...
                        transforms(:, :, indices), indices, heightRange, ikSeed, ...
                        ikAttempts, positionTolerance, orientationTolerance, ...
                        progressQueue, reportEnabled, showSolverStages);
                end
                futureCleanup = onCleanup(@() cancel(futures));
                heartbeatTimer = tic;
                fprintf('Submitted %d worker batches; waiting for worker messages...\n', numel(futures));
                while true
                    [message, available] = poll(progressQueue, 0.1);
                    if available
                        progressCallback(message);
                    end
                    drawnow;
                    for taskIndex = 1:numel(futures)
                        if strcmp(futures(taskIndex).State, 'finished') && ~isempty(futures(taskIndex).Error)
                            fetchOutputs(futures(taskIndex));
                        end
                    end
                    if toc(heartbeatTimer) >= 5 && reportEnabled
                        fprintf('Waiting: started %d, completed %d/%d, running %d, elapsed %.1f s.\n', ...
                            startedCount, completedCount, n, startedCount - completedCount, toc(solveTimer));
                        heartbeatTimer = tic;
                    end
                    if all(strcmp({futures.State}, 'finished'))
                        break;
                    end
                end
                for batch = 1:numel(futures)
                    output = fetchOutputs(futures(batch)); % Propagate worker errors.
                    indices = output.indices;
                    success(indices) = output.success;
                    configurations(indices, :) = output.q;
                    positionError(indices) = output.positionError;
                    orientationError(indices) = output.orientationError;
                    exitFlag(indices) = output.exitFlag;
                    attempts(indices) = output.attempts;
                    baseHeights(indices) = output.baseHeight;
                end
                while progressQueue.QueueLength > 0
                    message = poll(progressQueue);
                    progressCallback(message);
                end
            else
                for k = 1:n
                    attemptCallback = [];
                    if reportEnabled
                        progressCallback([k, 0, 0, NaN, NaN, 0, NaN]);
                        if showSolverStages
                            attemptCallback = localIKReporter(progressCallback, k);
                        end
                    end
                    stream = RandStream('Threefry', 'Seed', ikSeed);
                    stream.Substream = k;
                    [q, info] = arm.inverse_lift_transform(transforms(:, :, k), heightRange, ...
                        'Attempts', ikAttempts, ...
                        'PositionTolerance', positionTolerance, ...
                        'OrientationTolerance', orientationTolerance, ...
                        'RandomStream', stream, 'ProgressCallback', attemptCallback);
                    success(k) = info.success;
                    configurations(k, :) = q;
                    positionError(k) = info.positionError;
                    orientationError(k) = info.orientationError;
                    exitFlag(k) = info.exitFlag;
                    attempts(k) = info.attempts;
                    baseHeights(k) = info.baseHeight;
                    if reportEnabled
                    progressCallback([k, 3, double(info.success), info.positionError, info.orientationError, info.attempts, info.exitFlag, info.baseHeight]);
                    end
                end
            end
            drawnow;
            result.solveTime = toc(solveTimer);
            result.usedParallel = useParallel;
            result.workers = workerLimit;
            result.ikSeed = options.IKSeed;
            result.success = success;
            result.jointConfigurations = configurations;
            result.positionError = positionError;
            result.orientationError = orientationError;
            result.exitFlag = exitFlag;
            result.attempts = attempts;
            result.baseHeight = baseHeights;
            result.baseHeightRange = heightRange;
            [positions, ~, groups] = unique(result.samples(:, 1:3), 'rows');
            result.positionsScene = positions;
            result.positionsBase = positions * obj.T_base_scene(1:3, 1:3).' + obj.T_base_scene(1:3, 4).';
            result.positionIndex = groups;
            result.successFraction = accumarray(groups, double(result.success), [], @mean);
            result.allSampledPosesPassed = all(result.success);
            % Keep the geometry used for this evaluation even if the object is reconfigured.
            result.T_base_scene = obj.T_base_scene;
            result.R_cylinder_tool = obj.R_cylinder_tool;
            result.p_cylinder_tool = obj.p_cylinder_tool;
            result.positionTolerance = options.PositionTolerance;
            result.orientationTolerance = options.OrientationTolerance;

            function reportProgress(message)
                % Runs on the client; message order follows completion, not sample index.
                % Columns: sample, phase, success, position, orientation, attempt, exit flag, height.
                if message(2) ~= 3
                    if message(2) == 0
                        startedCount = startedCount + 1;
                    end
                    if showSolverStages && (completedCount == 0 || printEachSample)
                        switch message(2)
                            case 0
                                fprintf('Sample %d: started (worker ready), elapsed %.1f s.\n', message(1), toc(solveTimer));
                            case 1
                                fprintf('Sample %d: IK attempt %d/%d started, elapsed %.1f s.\n', ...
                                    message(1), message(6), ikAttempts, toc(solveTimer));
                            case 2
                                fprintf('Sample %d: IK attempt %d/%d finished, exit flag %g, elapsed %.1f s.\n', ...
                                    message(1), message(6), ikAttempts, message(7), toc(solveTimer));
                        end
                    end
                    return;
                end
                completedCount = completedCount + 1;
                passedCount = passedCount + message(3);
                if printEachSample
                    status = 'FAIL';
                    if message(3)
                        status = 'PASS';
                    end
                    fprintf('[%d/%d] Sample %d: %s, position %.6g mm, orientation %.6g deg, base height %.3f mm.\n', ...
                        completedCount, n, message(1), status, message(4) * 1000, rad2deg(message(5)), message(8) * 1000);
                end
                if progressEvery > 0 && (completedCount == 1 || mod(completedCount, progressEvery) == 0 || completedCount == n)
                    fprintf('Progress: %d/%d (%.1f%%), passed %d, failed %d, elapsed %.1f s.\n', ...
                        completedCount, n, 100 * completedCount / n, passedCount, ...
                        completedCount - passedCount, toc(solveTimer));
                end
            end
        end

        function ax = draw_cloud(~, result, ax)
            % A cylinder centre passes only when all its sampled orientations pass.
            arguments
                ~
                result (1, 1) struct
                ax = []
            end
            if isempty(ax)
                fig = figure('Name', 'Cylinder Pose Reachability', 'NumberTitle', 'off');
                ax = axes(fig);
            end
            category = double(result.successFraction > 0) + double(result.successFraction == 1);
            scatter3(ax, result.positionsScene(:, 1), result.positionsScene(:, 2), ...
                result.positionsScene(:, 3), 36, category, 'filled');
            colormap(ax, [0.8, 0.15, 0.15; 0.95, 0.7, 0.15; 0.1, 0.65, 0.25]);
            clim(ax, [-0.5, 2.5]);
            bar = colorbar(ax);
            bar.Ticks = [0, 1, 2];
            bar.TickLabels = {'None', 'Some', 'All'};
            bar.Label.String = 'Sampled orientations with a valid IK solution';
            grid(ax, 'on');
            axis(ax, 'equal');
            xlabel(ax, 'Scene X (m)');
            ylabel(ax, 'Scene Y (m)');
            zlabel(ax, 'Scene Z (m)');
            title(ax, sprintf('Cylinder centres: %d / %d sampled poses passed', ...
                nnz(result.success), numel(result.success)));
        end

        function draw_reference(obj, arm, ax, heightRange)
            arguments
                obj
                arm ArmModel
                ax (1, 1) matlab.graphics.axis.Axes
                heightRange (1, 2) double {mustBeReal, mustBeFinite, mustBeNonnegative} = [0.290, 0.490]
            end
            wasHeld = ishold(ax);
            hold(ax, 'on');
            joints = arm.get_joint_positions();
            referenceHeight = mean(heightRange);
            sceneBase = inv(obj.T_base_scene);
            sceneBase(3, 4) = referenceHeight;
            joints = joints * sceneBase(1:3, 1:3).' + sceneBase(1:3, 4).';
            plot3(ax, joints(:, 1), joints(:, 2), joints(:, 3), '-o', ...
                'Color', [0.15, 0.2, 0.3], 'LineWidth', 2, ...
                'MarkerFaceColor', [0.6, 0.65, 0.75], 'Tag', 'ArmSkeleton');
            plot3(ax, [0.1, 0.1], [0, 0], heightRange, 'k--', ...
                'LineWidth', 3, 'Tag', 'BaseLiftRange');
            text(ax, 0.1, 0, heightRange(2) + 0.02, ...
                sprintf('Lift %.0f-%.0f mm', heightRange * 1000));
            transforms = cat(3, sceneBase, eye(4));
            names = {'baselink (reference height)', 'Scene O'''};
            axisNames = {'X', 'Y', 'Z'};
            colors = [0.85, 0.15, 0.15; 0.1, 0.6, 0.2; 0.15, 0.3, 0.85];
            axisLength = 0.08;
            for frame = 1:2
                origin = transforms(1:3, 4, frame);
                plot3(ax, origin(1), origin(2), origin(3), 'ks', ...
                    'MarkerFaceColor', 'k', 'MarkerSize', 6, 'Tag', names{frame});
                text(ax, origin(1), origin(2), origin(3) - 0.025, names{frame}, ...
                    'FontWeight', 'bold', 'Interpreter', 'none');
                for k = 1:3
                    direction = axisLength * transforms(1:3, k, frame);
                    quiver3(ax, origin(1), origin(2), origin(3), ...
                        direction(1), direction(2), direction(3), 0, ...
                        'Color', colors(k, :), 'LineWidth', 1.5);
                    tip = origin + 1.15 * direction;
                    text(ax, tip(1), tip(2), tip(3), axisNames{k}, 'Color', colors(k, :));
                end
            end
            axis(ax, 'equal');
            axis(ax, 'padded');
            view(ax, 3);
            if ~wasHeld
                hold(ax, 'off');
            end
        end
    end
end

% These factories capture only their arguments, never evaluate's client workspace.
function callback = queueIKReporter(queue, index)
    callback = @(event) send(queue, [index, event]);
end

function callback = localIKReporter(reporter, index)
    callback = @(event) reporter([index, event]);
end

function output = solveLiftBatch(workerArm, transforms, indices, heightRange, ikSeed, ...
        attempts, positionTolerance, orientationTolerance, progressQueue, reportEnabled, showSolverStages)
    localArm = workerArm.Value;
    count = numel(indices);
    output.indices = indices;
    output.success = false(count, 1);
    output.q = nan(count, localArm.get_joint_count());
    output.positionError = inf(count, 1);
    output.orientationError = inf(count, 1);
    output.exitFlag = nan(count, 1);
    output.attempts = zeros(count, 1);
    output.baseHeight = nan(count, 1);
    for localIndex = 1:count
        sampleIndex = indices(localIndex);
        attemptCallback = [];
        if reportEnabled
            send(progressQueue, [sampleIndex, 0, 0, NaN, NaN, 0, NaN]);
            if showSolverStages
                attemptCallback = queueIKReporter(progressQueue, sampleIndex);
            end
        end
        stream = RandStream('Threefry', 'Seed', ikSeed);
        stream.Substream = sampleIndex;
        [q, info] = localArm.inverse_lift_transform(transforms(:, :, localIndex), heightRange, ...
            'Attempts', attempts, ...
            'PositionTolerance', positionTolerance, ...
            'OrientationTolerance', orientationTolerance, ...
            'RandomStream', stream, 'ProgressCallback', attemptCallback);
        output.success(localIndex) = info.success;
        output.q(localIndex, :) = q;
        output.positionError(localIndex) = info.positionError;
        output.orientationError(localIndex) = info.orientationError;
        output.exitFlag(localIndex) = info.exitFlag;
        output.attempts(localIndex) = info.attempts;
        output.baseHeight(localIndex) = info.baseHeight;
        if reportEnabled
            send(progressQueue, [sampleIndex, 3, double(info.success), info.positionError, ...
                info.orientationError, info.attempts, info.exitFlag, info.baseHeight]);
        end
    end
end
