%Arm model class for the Alliance robot arm

classdef ArmModel < handle

    methods (Access = public)

        function obj = ArmModel()
            obj.robot = obj.arm_init();
            obj.robot.plot(obj.q);
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
            target_position = SE3(target_position(1), target_position(2), target_position(3)) * SE3.Rz(target_position(6)) * SE3.Ry(target_position(5)) * SE3.Rx(target_position(4));
            obj.q = obj.arm_inverse(target_position);
        end

        function angle_controller(obj, angle, unit)

            switch unit
                case "deg"
                    angle = angle * obj.deg;
                case "rad"
                otherwise
                    error("unit must be deg or rad");
            end

            obj.q = angle;
        end

    end

    methods (Access = private)

        %configuration of the arm and initialization
        function arm = arm_config(obj)
            L(1) = Link('d', 0.05985, 'a', 0, 'alpha', pi / 2, 'offset', 0);
            L(2) = Link('d', 0, 'a', 0.37, 'alpha', 0, 'offset', pi / 2);
            L(3) = Link('d', 0, 'a', 0.04307, 'alpha', pi / 2, 'offset', 0);
            L(4) = Link('d', 0.40969, 'a', 0, 'alpha', pi / 2, 'offset', 0);
            L(5) = Link('d', 0, 'a', 0, 'alpha', pi / 2, 'offset', pi / 2);
            L(6) = Link('d', -0.1551, 'a', 0, 'alpha', 0, 'offset', 0);

            L(1).qlim = [-pi, pi];
            L(2).qlim = [-60 * obj.deg, 80 * obj.deg];
            L(3).qlim = [-70 * obj.deg, (30) * obj.deg];
            L(4).qlim = [-pi, pi];
            L(5).qlim = [-pi * 5/9 - 90 * obj.deg, pi * 5/9 - 90 * obj.deg];
            L(6).qlim = [-pi, pi];

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

        function angle = arm_inverse(obj, position)

            angle = obj.robot.ikcon(position, obj.q); %unit: rad

            disp("Inverse kinematics angle (rad): ");
            disp(angle);

        end

    end

    properties (Access = private)
        robot;
        q = [0, 0, 0, 0, 0, 0];

        is_first_show = true;

    end

    properties (Access = private, Constant)
        deg = pi / 180;
        rad = 180 / pi;
    end

end
