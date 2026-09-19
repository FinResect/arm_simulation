classdef TrajectoryDrawer < handle

    properties (SetAccess = private)
        Figure
        Axes
    end

    methods
        function obj = TrajectoryDrawer()
            obj.Figure = figure("Name", "End-Effector Trajectory", ...
                "NumberTitle", "off");
            obj.Axes = axes(obj.Figure);
        end

        function draw_trajectory(obj, trajectory)
            arguments
                obj
                trajectory double
            end

            if isempty(trajectory)
                cla(obj.Axes);
                title(obj.Axes, "No trajectory");
                return;
            end
            if size(trajectory, 2) ~= 3 || any(~isfinite(trajectory), "all")
                error("TrajectoryDrawer:InvalidTrajectory", ...
                    "The trajectory must be a finite N-by-3 matrix.");
            end

            cla(obj.Axes);
            plot3(obj.Axes, trajectory(:, 1), trajectory(:, 2), trajectory(:, 3), ...
                "LineWidth", 1.5, "Color", [0.1, 0.35, 0.8]);
            hold(obj.Axes, "on");
            scatter3(obj.Axes, trajectory(1, 1), trajectory(1, 2), trajectory(1, 3), ...
                48, [0.1, 0.65, 0.25], "filled");
            scatter3(obj.Axes, trajectory(end, 1), trajectory(end, 2), trajectory(end, 3), ...
                48, [0.85, 0.2, 0.15], "filled");
            hold(obj.Axes, "off");

            grid(obj.Axes, "on");
            axis(obj.Axes, "equal");
            xlabel(obj.Axes, "X (m)");
            ylabel(obj.Axes, "Y (m)");
            zlabel(obj.Axes, "Z (m)");
            title(obj.Axes, "End-Effector Trajectory");
            legend(obj.Axes, "Path", "Start", "End", "Location", "best");
            drawnow;
        end
    end
end
