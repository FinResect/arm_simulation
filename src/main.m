clear
clc
close all
clear classes

addpath('model/')
addpath('planning/')
addpath('draw/')

arm = ArmModel();
planner = RRTClass(arm, RRTMode.RRT);
drawer = TrajectoryDrawer();

planner.set_drawer(drawer);

targetPose = [-0.5, 0.3, 0.0, 3.1, 0.0, 0.5];
isStateValid = @(q) true;

result = planner.start_planning(targetPose, isStateValid);

if result.success
    % drawer.draw_trajectory(result.endEffectorPath);
    arm.execute_path(result.jointPath);
end
