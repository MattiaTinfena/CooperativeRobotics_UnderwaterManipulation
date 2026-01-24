classdef TaskNotMoving < Task
    properties

    end

    methods
        function updateReference(obj, robot)
            obj.xdotbar = zeros(6,1);
        end

        function updateJacobian(obj, robot)

            Jt_a  = zeros(6,7);

            wRv = robot.wTv(1:3, 1:3);
            Jt_v = [-wRv, zeros(3); zeros(3), -wRv];

            obj.J = [Jt_a Jt_v];
        end

        function updateActivation(obj, robot)
            obj.A = eye(6);
        end
    end
end