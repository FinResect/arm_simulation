clear
clc
close all
clear classes

arm = ArmModel();
fig = gcf;
target = [0.5, 0.2, 0.2, 3.1, 0.0, 0.5];

while isgraphics(fig)

    arm.position_controller(target);

    % arm.angle_controller(target, "deg");
    target(1) = target(1) + 0.01;
    arm.arm_show();
    pause(0.001);
end
