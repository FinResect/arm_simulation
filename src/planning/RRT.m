classdef RRT < handle

    properties (SetAccess = private)
        arm
        qStart
        qLim
    end

    properties (Access = private)
        maxIterations_ = 5000
        stepSize_ = 0.15
        goalBias_ = 0.1
        goalTolerance_ = 0.05
        edgeResolution_ = 0.05
    end

    methods
        function obj = RRT(arm, options)
            arguments
                arm ArmModel
                options.MaxIterations (1, 1) double {mustBeInteger, mustBePositive} = 5000
                options.StepSize (1, 1) double {mustBePositive} = 0.15
                options.GoalBias (1, 1) double {mustBeGreaterThanOrEqual(options.GoalBias, 0), mustBeLessThanOrEqual(options.GoalBias, 1)} = 0.1
                options.GoalTolerance (1, 1) double {mustBePositive} = 0.05
                options.EdgeResolution (1, 1) double {mustBePositive} = 0.05
            end

            obj.arm = arm;
            obj.refresh_model_state();
            obj.maxIterations_ = options.MaxIterations;
            obj.stepSize_ = options.StepSize;
            obj.goalBias_ = options.GoalBias;
            obj.goalTolerance_ = options.GoalTolerance;
            obj.edgeResolution_ = options.EdgeResolution;
        end

        function result = plan(obj, qGoal, isStateValid)
            arguments
                obj
                qGoal double
                isStateValid function_handle = @(q) true
            end

            obj.refresh_model_state();
            qGoal = reshape(qGoal, 1, []);
            obj.validate_configuration(qGoal, "qGoal");

            if ~obj.is_valid_state(obj.qStart, isStateValid)
                error("RRT:InvalidStart", "The current arm configuration is invalid.");
            end
            if ~obj.is_valid_state(qGoal, isStateValid)
                error("RRT:InvalidGoal", "The goal arm configuration is invalid.");
            end

            nodes = obj.qStart;
            parents = 0;
            goalIndex = 0;
            iterations = 0;
            timerValue = tic;

            for iteration = 1:obj.maxIterations_
                iterations = iteration;
                sample = obj.sample(qGoal);
                nearestIndex = obj.nearest(nodes, sample);
                candidate = obj.steer(nodes(nearestIndex, :), sample);

                if ~obj.is_valid_edge(nodes(nearestIndex, :), candidate, isStateValid)
                    continue;
                end

                nodes(end + 1, :) = candidate; %#ok<AGROW>
                parents(end + 1) = nearestIndex; %#ok<AGROW>
                candidateIndex = size(nodes, 1);

                if norm(candidate - qGoal) <= obj.goalTolerance_ && ...
                        obj.is_valid_edge(candidate, qGoal, isStateValid)
                    nodes(end + 1, :) = qGoal; %#ok<AGROW>
                    parents(end + 1) = candidateIndex; %#ok<AGROW>
                    goalIndex = size(nodes, 1);
                    break;
                end
            end

            if goalIndex == 0
                path = zeros(0, numel(obj.qStart));
                success = false;
            else
                path = obj.backtrack(nodes, parents, goalIndex);
                success = true;
            end

            result = struct( ...
                "success", success, ...
                "jointPath", path, ...
                "iterations", iterations, ...
                "nodes", size(nodes, 1), ...
                "elapsedTime", toc(timerValue), ...
                "qStart", obj.qStart, ...
                "qGoal", qGoal);
        end
    end

    methods (Access = private)
        function refresh_model_state(obj)
            obj.qStart = reshape(obj.arm.get_current_configuration(), 1, []);
            obj.qLim = obj.arm.get_joint_limits();

            if size(obj.qLim, 1) ~= 2
                obj.qLim = obj.qLim.';
            end
            obj.validate_configuration(obj.qStart, "current configuration");
            if size(obj.qLim, 2) ~= numel(obj.qStart)
                error("RRT:InvalidJointLimits", "Joint limits do not match the arm degrees of freedom.");
            end
        end

        function validate_configuration(obj, q, name)
            if numel(q) ~= 6
                error("RRT:InvalidConfiguration", "%s must contain six joint angles.", name);
            end
            if any(q < obj.qLim(1, :) | q > obj.qLim(2, :))
                error("RRT:OutOfBounds", "%s is outside the joint limits.", name);
            end
        end

        function sample = sample(obj, qGoal)
            if rand() < obj.goalBias_
                sample = qGoal;
            else
                sample = obj.qLim(1, :) + rand(1, 6) .* (obj.qLim(2, :) - obj.qLim(1, :));
            end
        end

        function index = nearest(~, nodes, sample)
            [~, index] = min(sum((nodes - sample).^2, 2));
        end

        function candidate = steer(obj, source, target)
            delta = target - source;
            distance = norm(delta);
            if distance <= obj.stepSize_
                candidate = target;
            else
                candidate = source + delta / distance * obj.stepSize_;
            end
            candidate = min(max(candidate, obj.qLim(1, :)), obj.qLim(2, :));
        end

        function valid = is_valid_state(obj, q, isStateValid)
            valid = all(q >= obj.qLim(1, :) & q <= obj.qLim(2, :));
            if valid
                result = isStateValid(q);
                valid = isscalar(result) && islogical(result) && result;
            end
        end

        function valid = is_valid_edge(obj, source, target, isStateValid)
            distance = norm(target - source);
            samples = max(1, ceil(distance / obj.edgeResolution_));
            valid = true;
            for index = 1:samples
                q = source + (target - source) * (index / samples);
                if ~obj.is_valid_state(q, isStateValid)
                    valid = false;
                    return;
                end
            end
        end

        function path = backtrack(~, nodes, parents, goalIndex)
            indices = goalIndex;
            while parents(indices(1)) ~= 0
                indices = [parents(indices(1)), indices]; %#ok<AGROW>
            end
            path = nodes(indices, :);
        end
    end
end
