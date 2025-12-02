classdef TaskZeroAltitude < Task   
    properties

    end

    methods
        function updateReference(obj, robot)
            if size(robot.altitude) == 1
                obj.xdotbar = 0.2 * robot.altitude;
            else
                obj.xdotbar = 0;
            end
            % limit the requested velocities...
            obj.xdotbar = Saturate(obj.xdotbar, 0.2);
        end
        function updateJacobian(obj, robot)
            n = [0 0 1];
            Jt_a  = n * zeros(3,7);
            wRv = robot.wTv(1:3, 1:3);
            Jt_v = n * [-(wRv) zeros(3)];
            obj.J = [Jt_a Jt_v];
        end
        
        function updateActivation(obj, robot)
            obj.A = 1;
        end
    end
end