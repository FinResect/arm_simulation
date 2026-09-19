classdef RRTClass < handle

    properties (SetAccess = private)
        arm
        mode
    end

    properties (Access = private)
        drawer_ = []
        planner_ = []
    end

    methods (Access = public)
        function obj = RRTClass(arm, mode)
            arguments
                arm ArmModel
                mode RRTMode = RRTMode.RRT
            end

            obj.arm = arm;
            obj.mode = mode;
        end

        function switch_mode(obj, mode)
            arguments
                obj
                mode RRTMode
            end

            obj.mode = mode;
        end

        function set_drawer(obj, drawer)
            arguments
                obj
                drawer
            end

            if ~ismethod(drawer, "draw_trajectory")
                error("RRTClass:InvalidDrawer", ...
                    "The drawer must provide a draw_trajectory(trajectory) method.");
            end
            obj.drawer_ = drawer;
        end

        function clear_drawer(obj)
            obj.drawer_ = [];
        end

        function result = start_planning(obj, targetPose, isStateValid, options)
            arguments
                obj
                targetPose (1, 6) double
                isStateValid function_handle = @(q) true
                options.MaxIterations (1, 1) double {mustBeInteger, mustBePositive} = 5000
                options.StepSize (1, 1) double {mustBePositive} = 0.15
                options.GoalBias (1, 1) double {mustBeGreaterThanOrEqual(options.GoalBias, 0), mustBeLessThanOrEqual(options.GoalBias, 1)} = 0.1
                options.GoalTolerance (1, 1) double {mustBePositive} = 0.05
                options.EdgeResolution (1, 1) double {mustBePositive} = 0.05
            end

            qGoal = obj.arm.inverse_position(targetPose);

            switch obj.mode
                case RRTMode.RRT
                    obj.planner_ = RRT(obj.arm, ...
                        "MaxIterations", options.MaxIterations, ...
                        "StepSize", options.StepSize, ...
                        "GoalBias", options.GoalBias, ...
                        "GoalTolerance", options.GoalTolerance, ...
                        "EdgeResolution", options.EdgeResolution);
                otherwise
                    error("RRTClass:UnsupportedMode", ...
                        "The selected planning mode is not implemented: %s.", string(obj.mode));
            end

            result = obj.planner_.plan(qGoal, isStateValid);
            result.targetPose = targetPose;
            result.endEffectorPath = obj.arm.get_end_effector_path(result.jointPath);
            obj.draw_result(result.endEffectorPath);
        end
    end

    methods (Access = private)
        function draw_result(obj, trajectory)
            if isempty(obj.drawer_)
                return;
            end
            obj.drawer_.draw_trajectory(trajectory);
        end
    end
end
