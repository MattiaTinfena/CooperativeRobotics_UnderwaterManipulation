classdef TaskVehicleOrientation < Task   
    properties

    end

    methods
        function updateReference(obj, robot)
            [ang,~] = CartError(robot.wTgv , robot.wTv); % I compute the cartesian error between two frames projected on w
            obj.xdotbar = 0.2 * ang;
            % limit the requested velocities...
            obj.xdotbar = Saturate(obj.xdotbar, 0.2);
        end
        function updateJacobian(obj, robot)
            Jt_a  = zeros(3,7);
            wRv = robot.wTv(1:3, 1:3);
            Jt_v = [zeros(3) (wRv)];
            obj.J = [Jt_a Jt_v];
        end
        
        function updateActivation(obj, robot)

            [ang,~] = CartError(robot.wTgv , robot.wTv);
            errorz = acos(ang(3)/norm(ang));
            errorz = min((pi - errorz), errorz);
            obj.A = zeros(3);
            obj.A(1,1) = IncreasingBellShapedFunction(0.1,0.2,0,1,abs(errorz));
            obj.A(2,2) = IncreasingBellShapedFunction(0.1,0.2,0,1,abs(errorz));
        end
    end
end