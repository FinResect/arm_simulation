%Arm model class for the Alliance robot arm

classdef ArmModel < handle

    methods (Access = public)

        function obj = ArmModel(options)

            arguments
                options.Show (1, 1) logical = true
            end

            obj.robot = obj.arm_init();

            if options.Show
                obj.robot.plot(obj.q);
            end

        end

        function arm_show(obj)

            if (obj.is_first_show)
                obj.robot.plot(obj.q);
                obj.is_first_show = false;
            end

            obj.robot.animate(obj.q);
            drawnow;

        end

        function position_controller(obj, target_position) %xyzrpy
            obj.q = obj.inverse_position(target_position);
        end

        function angle_controller(obj, angle, unit)

            if ~isnumeric(angle) || ~isvector(angle)
                error("ArmModel:InvalidConfiguration", "Joint angles must be a numeric vector.");
            end

            switch unit
                case "deg"
                    angle = angle * obj.deg;
                case "rad"
                otherwise
                    error("unit must be deg or rad");
            end

            angle = reshape(angle, 1, []);
            obj.validate_joint_array(angle);
            obj.q = angle;
        end

        function angle = get_current_configuration(obj)
            angle = obj.q;
        end

        function count = get_joint_count(obj)
            count = obj.robot.n;
        end

        function positions = get_joint_positions(obj)
            transform = obj.robot.base;
            positions = zeros(obj.get_joint_count() + 1, 3);
            positions(1, :) = transform.t.';

            for k = 1:obj.get_joint_count()
                transform = transform * obj.robot.links(k).A(obj.q(k));
                positions(k + 1, :) = transform.t.';
            end

        end

        function limits = get_joint_limits(obj)
            limits = obj.robot.qlim;

            if size(limits, 1) ~= 2
                limits = limits.';
            end

        end

        function angle = inverse_position(obj, target_position)

            arguments
                obj
                target_position (1, 6) double {mustBeReal, mustBeFinite}
            end

            target_transform = obj.position_to_transform(target_position);
            [angle, info] = obj.inverse_transform(target_transform.T);

            if ~info.success
                error("ArmModel:InverseKinematicsFailed", ...
                "Inverse kinematics did not converge to a valid joint configuration.");
            end

        end

        function transform = forward_transform(obj, q)

            arguments
                obj
                q double
            end

            if ~isvector(q)
                error("ArmModel:InvalidConfiguration", "Joint angles must be a vector.");
            end

            q = reshape(q, 1, []);
            obj.validate_joint_array(q);
            pose = obj.robot.fkine(q);
            transform = pose.T;

        end

        function [angle, info] = inverse_transform(obj, target, options)

            arguments
                obj
                target (4, 4) double {mustBeReal, mustBeFinite}
                options.Attempts (1, 1) double {mustBeReal, mustBeFinite, mustBeInteger, mustBePositive} = 3
                options.PositionTolerance (1, 1) double {mustBeReal, mustBeFinite, mustBePositive} = 1e-4
                options.OrientationTolerance (1, 1) double {mustBeReal, mustBeFinite, mustBePositive} = 1e-3
                options.RandomStream = []
                options.ProgressCallback = []
            end

            if ~isempty(options.RandomStream) && ~isa(options.RandomStream, 'RandStream')
                error('ArmModel:InvalidRandomStream', 'RandomStream must be a RandStream object or empty.');
            end
            if ~isempty(options.ProgressCallback) && ~isa(options.ProgressCallback, 'function_handle')
                error('ArmModel:InvalidProgressCallback', 'ProgressCallback must be a function handle or empty.');
            end

            rotation = target(1:3, 1:3);

            if norm(target(4, :) - [0, 0, 0, 1]) > 1e-10 || ...
                    norm(rotation.' * rotation - eye(3), 'fro') > 1e-8 || ...
                    abs(det(rotation) - 1) > 1e-8
                error("ArmModel:InvalidTransform", "The target must be a proper rigid transform.");
            end

            count = obj.get_joint_count();
            limits = obj.get_joint_limits();
            angle = nan(1, count);
            info = struct("success", false, "positionError", Inf, ...
                "orientationError", Inf, "exitFlag", NaN, "attempts", 0);
            bestError = Inf;
            targetPose = SE3(target);

            for attempt = 1:options.Attempts

                if attempt == 1
                    seed = obj.q;
                elseif attempt == 2
                    seed = mean(limits, 1);
                else

                    if isempty(options.RandomStream)
                        randomValues = rand(1, count);
                    else
                        randomValues = rand(options.RandomStream, 1, count);
                    end

                    seed = limits(1, :) + randomValues .* (limits(2, :) - limits(1, :));
                end

                if ~isempty(options.ProgressCallback)
                    options.ProgressCallback([1, 0, NaN, NaN, attempt, NaN]);
                end
                % Progress callbacks stay outside evalc so their output is never captured.
                % Some RTB releases echo exitflag_ despite Display='off'.
                solverOutput = evalc('[candidate, ~, exitFlag] = obj.robot.ikcon(targetPose, seed);');
                solverOutput = regexprep(solverOutput, ...
                    '(^|\n)exitflag_ =\s*[-+]?\d+\s*(?=\n|$)', '');
                if ~isempty(strtrim(solverOutput))
                    fprintf('%s', solverOutput);
                end
                info.attempts = attempt;
                if ~isempty(options.ProgressCallback)
                    options.ProgressCallback([2, 0, NaN, NaN, attempt, exitFlag]);
                end

                if ~isnumeric(candidate) || ~isreal(candidate) || ...
                        ~isvector(candidate) || numel(candidate) ~= count || any(~isfinite(candidate(:)))
                    continue;
                end

                candidate = reshape(candidate, 1, []);
                pose = obj.robot.fkine(candidate);
                actual = pose.T;
                positionError = norm(actual(1:3, 4) - target(1:3, 4));
                cosine = (trace(rotation.' * actual(1:3, 1:3)) - 1) / 2;
                orientationError = acos(max(-1, min(1, cosine)));

                if any(~isfinite(actual(:))) || ~isfinite(positionError) || ~isfinite(orientationError)
                    continue;
                end

                success = isscalar(exitFlag) && isreal(exitFlag) && isfinite(exitFlag) && exitFlag > 0 && ...
                    all(candidate >= limits(1, :) & candidate <= limits(2, :)) && ...
                    positionError <= options.PositionTolerance && orientationError <= options.OrientationTolerance;
                % Compare both residuals relative to their requested tolerances.
                residual = hypot(positionError / options.PositionTolerance, ...
                    orientationError / options.OrientationTolerance);

                if ~isfinite(info.positionError) || residual < bestError || success
                    bestError = residual;
                    info.positionError = positionError;
                    info.orientationError = orientationError;
                    info.exitFlag = exitFlag;
                end

                if success
                    angle = candidate;
                    info.success = true;
                    return;
                end

            end

        end

        function [angle, info] = inverse_lift_transform(obj, targetScene, heightRange, options)

            arguments
                obj
                targetScene (4, 4) double {mustBeReal, mustBeFinite}
                heightRange (1, 2) double {mustBeReal, mustBeFinite, mustBeNonnegative} = [0.290, 0.490]
                options.Attempts (1, 1) double {mustBeReal, mustBeFinite, mustBeInteger, mustBePositive} = 3
                options.PositionTolerance (1, 1) double {mustBeReal, mustBeFinite, mustBePositive} = 1e-4
                options.OrientationTolerance (1, 1) double {mustBeReal, mustBeFinite, mustBePositive} = 1e-3
                options.RandomStream = []
                options.ProgressCallback = []
            end

            if ~isempty(options.RandomStream) && ~isa(options.RandomStream, 'RandStream')
                error('ArmModel:InvalidRandomStream', 'RandomStream must be a RandStream object or empty.');
            end
            if ~isempty(options.ProgressCallback) && ~isa(options.ProgressCallback, 'function_handle')
                error('ArmModel:InvalidProgressCallback', 'ProgressCallback must be a function handle or empty.');
            end

            rotation = targetScene(1:3, 1:3);
            if norm(targetScene(4, :) - [0, 0, 0, 1]) > 1e-10 || ...
                    norm(rotation.' * rotation - eye(3), 'fro') > 1e-8 || ...
                    abs(det(rotation) - 1) > 1e-8
                error("ArmModel:InvalidTransform", "The target must be a proper rigid transform.");
            end
            if heightRange(1) > heightRange(2)
                error("ArmModel:InvalidHeightRange", "Height bounds must be ordered from minimum to maximum.");
            end

            count = obj.get_joint_count();
            limits = obj.get_joint_limits();
            if ~isnumeric(limits) || ~isreal(limits) || ~isequal(size(limits), [2, count]) || ...
                    any(~isfinite(limits(:))) || any(limits(1, :) > limits(2, :))
                error("ArmModel:InvalidJointLimits", "Joint bounds must be finite, real, and ordered.");
            end
            obj.validate_joint_array(obj.q);
            lower = [limits(1, :), heightRange(1)];
            width = [limits(2, :), heightRange(2)] - lower;
            scale = width;
            scale(scale == 0) = 1;
            % Fixed physical variables remain fixed at zero in normalized space.
            upperScaled = double(width > 0);
            lengthScale = max(eps, sum(abs([obj.robot.links.a])) + sum(abs([obj.robot.links.d])));
            objective = @(x) lift_transform_objective(x, obj.robot, targetScene, lower, width, lengthScale);
            solverOptions = optimoptions('fmincon', 'Algorithm', 'sqp', ...
                'Display', 'off', 'UseParallel', false, 'MaxIterations', 1000, ...
                'MaxFunctionEvaluations', 20000, 'OptimalityTolerance', 1e-12, ...
                'StepTolerance', 1e-12, 'ConstraintTolerance', 1e-12);
            angle = nan(1, count);
            info = struct("success", false, "positionError", Inf, ...
                "orientationError", Inf, "exitFlag", NaN, "attempts", 0, "baseHeight", NaN);
            bestError = Inf;

            for attempt = 1:options.Attempts
                if attempt == 1
                    seed = [obj.q, mean(heightRange)];
                elseif attempt == 2
                    seed = [mean(limits, 1), mean(heightRange)];
                else
                    if isempty(options.RandomStream)
                        randomValues = rand(1, count + 1);
                    else
                        randomValues = rand(options.RandomStream, 1, count + 1);
                    end
                    seed = lower + randomValues .* width;
                end
                seedScaled = (seed - lower) ./ scale;

                if ~isempty(options.ProgressCallback)
                    options.ProgressCallback([1, 0, NaN, NaN, attempt, NaN]);
                end
                [candidateScaled, ~, exitFlag] = fmincon(objective, seedScaled, ...
                    [], [], [], [], zeros(1, count + 1), upperScaled, [], solverOptions);
                info.attempts = attempt;
                if ~isempty(options.ProgressCallback)
                    options.ProgressCallback([2, 0, NaN, NaN, attempt, exitFlag]);
                end

                if ~isnumeric(candidateScaled) || ~isreal(candidateScaled) || ...
                        ~isvector(candidateScaled) || numel(candidateScaled) ~= count + 1 || ...
                        any(~isfinite(candidateScaled(:)))
                    continue;
                end
                candidate = lower + reshape(candidateScaled, 1, []) .* width;
                qCandidate = candidate(1:count);
                height = candidate(end);
                baseScene = [-1, 0, 0, 0.1; 0, -1, 0, 0; 0, 0, 1, -height; 0, 0, 0, 1];
                pose = obj.robot.fkine(qCandidate);
                actual = baseScene \ pose.T;
                positionError = norm(actual(1:3, 4) - targetScene(1:3, 4));
                cosine = (trace(rotation.' * actual(1:3, 1:3)) - 1) / 2;
                orientationError = acos(max(-1, min(1, cosine)));

                if any(~isfinite(actual(:))) || ~isfinite(positionError) || ~isfinite(orientationError)
                    continue;
                end
                success = isscalar(exitFlag) && isreal(exitFlag) && isfinite(exitFlag) && exitFlag > 0 && ...
                    all(qCandidate >= limits(1, :) & qCandidate <= limits(2, :)) && ...
                    height >= heightRange(1) && height <= heightRange(2) && ...
                    positionError <= options.PositionTolerance && orientationError <= options.OrientationTolerance;
                residual = hypot(positionError / options.PositionTolerance, ...
                    orientationError / options.OrientationTolerance);
                if ~isfinite(info.positionError) || residual < bestError || success
                    bestError = residual;
                    info.positionError = positionError;
                    info.orientationError = orientationError;
                    info.exitFlag = exitFlag;
                end
                if success
                    angle = qCandidate;
                    info.success = true;
                    info.baseHeight = height;
                    return;
                end
            end

        end

        function execute_path(obj, path, options)

            arguments
                obj
                path double
                options.Pause (1, 1) double {mustBeNonnegative} = 0.01
                options.Show (1, 1) logical = true
            end

            obj.validate_joint_array(path);

            for index = 1:size(path, 1)
                obj.angle_controller(path(index, :), "rad");

                if options.Show
                    obj.arm_show();
                end

                if options.Pause > 0
                    pause(options.Pause);
                end

            end

        end

        function trajectory = get_end_effector_path(obj, path)

            arguments
                obj
                path double
            end

            obj.validate_joint_array(path);
            trajectory = zeros(size(path, 1), 3);

            for index = 1:size(path, 1)
                transform = obj.robot.fkine(path(index, :));
                trajectory(index, :) = transform.t.';
            end

        end

    end

    methods (Access = private)

        function validate_joint_array(obj, angles)

            if isequal(size(angles), [0, 0])
                return;
            end

            if ~ismatrix(angles) || ~isreal(angles) || size(angles, 2) ~= obj.get_joint_count() || ...
                    any(~isfinite(angles(:)))
                error("ArmModel:InvalidConfiguration", ...
                    "Joint configurations must be a finite real N-by-%d matrix.", obj.get_joint_count());
            end

            limits = obj.get_joint_limits();

            if any(any(angles < limits(1, :) | angles > limits(2, :)))
                error("ArmModel:OutOfBounds", "Joint configurations must be within the joint limits.");
            end

        end

        function transform = position_to_transform(~, target_position)
            transform = SE3(target_position(1), target_position(2), target_position(3)) ...
                * SE3.Rz(target_position(6)) ...
                * SE3.Ry(target_position(5)) ...
                * SE3.Rx(target_position(4));
        end

        %configuration of the arm and initialization
        function arm = arm_config(obj)
            L(1) = Link('d', 0.056, 'a', 0, 'alpha', -pi / 2, 'offset', 0);
            L(2) = Link('d', 0, 'a', 0, 'alpha', pi / 2, 'offset', 0);
            L(3) = Link('d', 0.24804 + 0.10903, 'a', 0.0655, 'alpha', -pi / 2, 'offset', 0);
            L(4) = Link('d', 0, 'a', 0, 'alpha', -pi / 2, 'offset', -pi / 2);
            L(5) = Link('d', 0.30508, 'a', 0, 'alpha', pi / 2, 'offset', pi / 2);
            L(6) = Link('d', 0, 'a', 0, 'alpha', pi / 2, 'offset', 0);
            L(7) = Link('d', 0, 'a', 0, 'alpha', 0, 'offset', pi / 2);

            L(1).qlim = [-170 * obj.deg, 170 * obj.deg];
            L(2).qlim = [-90 * obj.deg, 30 * obj.deg];
            L(3).qlim = [-166 * obj.deg, 160 * obj.deg];
            L(4).qlim = [-30 * obj.deg, 90 * obj.deg];
            L(5).qlim = [-pi, pi];
            L(6).qlim = [-pi * 5/9 - 90 * obj.deg, pi * 5/9 + 90 * obj.deg];
            L(7).qlim = [-pi, pi];

            % L(1).qlim = [-pi, pi];
            % L(2).qlim = [-pi, pi];
            % L(3).qlim = [-pi, pi];
            % L(4).qlim = [-pi, pi];
            % L(5).qlim = [-pi, pi];
            % L(6).qlim = [-pi, pi];
            % L(7).qlim = [-pi, pi];

            arm = SerialLink(L, 'name', 'Alliance');
        end

        function arm = arm_init(obj)
            arm = obj.arm_config();
        end

        %kinematics
        function position = arm_forward(obj, angle, unit)

            switch unit
                case "deg"
                    angle = angle * obj.deg;
                case "rad"
                otherwise
                    error("unit must be deg or rad");
            end

            position = obj.robot.fkine(angle);

            disp("Forward kinematics position (deg): ");
            disp(position);

        end

    end

    properties (Access = private)
        robot;
        q = [0, 0, 0, 0, 0, -pi / 2, 0];

        is_first_show = true;

    end

    properties (Access = private, Constant)
        deg = pi / 180;
        rad = 180 / pi;
    end

end

function value = lift_transform_objective(x, robot, targetScene, lower, width, lengthScale)
    candidate = lower + reshape(x, 1, []) .* width;
    height = candidate(end);
    baseScene = [-1, 0, 0, 0.1; 0, -1, 0, 0; 0, 0, 1, -height; 0, 0, 0, 1];
    pose = robot.fkine(candidate(1:end-1));
    actual = baseScene \ pose.T;
    translationResidual = (actual(1:3, 4) - targetScene(1:3, 4)) / lengthScale;
    rotationResidual = actual(1:3, 1:3) - targetScene(1:3, 1:3);
    value = sum(translationResidual .^ 2) + 0.5 * norm(rotationResidual, 'fro') ^ 2;
end
