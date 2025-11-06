classdef TaskVehicleAltitude < Task   
    properties

    end

    methods
        function updateReference(obj, robot)
            [~,lin] = CartError(robot.wTv, eye(4)); % I compute the cartesian error between two frames projected on w
            obj.xdotbar = 0.2 * lin;
            % limit the requested velocities...
            obj.xdotbar = Saturate(obj.xdotbar, 0.2);
        end
        function updateJacobian(obj, robot)
            Jt_a  = zeros(3,7);
            wRv = robot.wTv(1:3, 1:3);
            Jt_v = [-(wRv) zeros(3)];
            obj.J = [Jt_a Jt_v];
        end
        
        function updateActivation(obj, robot)

            obj.A = zeros(3); 
            if size(robot.altitude) == 1
                obj.A(3,3) = DecreasingBellShapedFunction(1,1.5,0,1,robot.altitude);
            end
        end
    end
end