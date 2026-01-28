classdef TaskMaxNoduleDist < Task
    properties

    end

    methods
        function updateReference(obj, robot)
            [~,lin] = CartError(robot.wTg, robot.wTv);
            n = norm(lin(1:2));
            lin_norm = lin / n;
            error_norm = n - 1.5;
            capped_norm = max(0, error_norm);
            xdot = lin_norm(1) * capped_norm;
            ydot =  lin_norm(2) * capped_norm;

            obj.xdotbar = - 0.2 * [xdot; ydot];
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

            n = norm(lin(1:2));
            lin_norm = lin / n;
            error_norm = n - 1.5;
            capped_norm = max(0, error_norm);
            xerror = lin_norm(1) * capped_norm;
            yerror =  lin_norm(2) * capped_norm;

            obj.A = eye(2);
            obj.A(1,1) = IncreasingBellShapedFunction(0,0.2,0,1,abs(xerror));
            obj.A(2,2) = IncreasingBellShapedFunction(0.0,0.2,0,1,abs(yerror));
        end
    end
end