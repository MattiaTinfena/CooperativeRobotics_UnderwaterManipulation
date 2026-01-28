classdef TaskVehicleHorAtt < Task
    properties
        rho
    end

    methods
        function updateReference(obj, robot)
            rho1 = [0; 0; 1];
            rho2 = robot.wTv(1:3,3);
            obj.rho = ReducedVersorLemma(rho1,rho2);
            robot.theta = norm(obj.rho);
            obj.xdotbar = 0.2 * (0.1 - robot.theta);
            % limit the requested velocities...
            obj.xdotbar = Saturate(obj.xdotbar, 0.2);
        end
        function updateJacobian(obj, robot)

            n = obj.rho / robot.theta;
            Jt_a  = n' * zeros(3,7);
            Jt_v = n' * [zeros(3) eye(3)];
            obj.J = [Jt_a Jt_v];
        end

        function updateActivation(obj, robot)

            obj.A = IncreasingBellShapedFunction(0.1,0.2,0,1,robot.theta);

        end
    end
end