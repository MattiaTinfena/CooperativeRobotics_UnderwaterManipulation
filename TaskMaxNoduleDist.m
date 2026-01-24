classdef TaskMaxNoduleDist < Task
    properties

    end

    methods
        function updateReference(obj, robot)
            [~,lin] = CartError(robot.wTg, robot.wTv);

            obj.xdotbar = - 0.2 * lin(1:2);
            obj.xdotbar = Saturate(obj.xdotbar, 0.2);
        end
        function updateJacobian(obj, robot)
            Jt_a  = zeros(2,7);
            wRv = robot.wTv(1:3, 1:3);
            J_full_v = [(-wRv) zeros(3)];
            Jt_v = J_full_v(1:2, :);
            obj.J = [Jt_a Jt_v];
        end

        function updateActivation(obj, robot)
            [~,lin] = CartError(robot.wTg, robot.wTv);
            dist = norm(lin(1:2));
            obj.A = eye(2) * IncreasingBellShapedFunction(1.5,1.7,0,1,dist);
        end
    end
end