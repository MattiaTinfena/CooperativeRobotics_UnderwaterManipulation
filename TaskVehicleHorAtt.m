classdef TaskVehicleHorAtt < Task   
    properties

    end

    methods
        function updateReference(obj, robot)
            rho1 = cross([0; 0; 1], robot.wTv(1:3,3));
            rho2 = dot([0; 0; 1], robot.wTv(1:3,3));
            theta = atan2(norm(rho1), rho2);
            obj.xdotbar = 0.2 * (0.1 - theta);
            % limit the requested velocities...
            obj.xdotbar = Saturate(obj.xdotbar, 0.2);
        end
        function updateJacobian(obj, robot)
            rho = cross([0; 0; 1], robot.wTv(1:3,3));
            theta = asin(norm(rho));
            n = rho / theta;
            Jt_a  = n' * zeros(3,7);
            wRv = robot.wTv(1:3, 1:3);
            Jt_v = n' * [zeros(3) (wRv)];
            obj.J = [Jt_a Jt_v];
        end
        
        function updateActivation(obj, robot)

            rho = cross([0; 0; 1], robot.wTv(1:3,3));
            theta = asin(norm(rho));
            obj.A = IncreasingBellShapedFunction(0.1,0.2,0,1,theta);

        end
    end
end