classdef TaskNotMoving < Task
    properties

    end

    methods
        function obj=TaskNotMoving()
            obj.instant_activation = true;
        end
        function updateReference(obj, robot)
            obj.xdotbar = zeros(6,1);
        end

        function updateJacobian(obj, robot)

            Jt_a  = zeros(6,7);
            Jt_v = [eye(3), zeros(3); zeros(3), eye(3)];
            obj.J = [Jt_a Jt_v];
        end

        function updateActivation(obj, robot)
            obj.A = eye(6);
        end
    end
end