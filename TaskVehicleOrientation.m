classdef TaskVehicleOrientation < Task
    properties

    end

    methods
        function updateReference(obj, robot)

            v_pos = robot.wTv(1:3, 4);
            n_pos = robot.wTg(1:3, 4);

            error_x = n_pos(1) - v_pos(1);
            error_y = n_pos(2) - v_pos(2);

            des_or = atan2(error_y, error_x);
            wRv_des = rotation(0, 0, des_or);
            wTv_des = [wRv_des, v_pos; 0 0 0 1];

            [ang, ~] = CartError(wTv_des, robot.wTv);

            obj.xdotbar = - 0.2 * ang;
            obj.xdotbar = Saturate(obj.xdotbar, 0.2);
        end

        function updateJacobian(obj, robot)

            Jt_a  = zeros(3,7);

            wRv = robot.wTv(1:3, 1:3);
            Jt_v = [zeros(3) (-wRv)];

            obj.J = [Jt_a Jt_v];
        end

        function updateActivation(obj, robot)
            obj.A = eye(3);
        end
    end
end