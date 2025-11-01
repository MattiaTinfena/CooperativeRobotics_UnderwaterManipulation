classdef TaskVehicle < Task   
    properties

    end

    % 1: x(c) = d = (Ogoal - Ovehicle)
    % 2: I want my d to go to 0
    % 3: Identify the time behaviour of d -> d_dot = v_g/w - v_v/w
    % 4: d_dot - v_g/w = - v_v/w (independent from vehicle = dep. from vehicle)
    % 5: I want to express the equation in matrix form:
    %       d_dot - v_g/w = v_J_d/w * y_dot, where 
    %           v_J_d/w = [0[3xl] -I[3x3] 0[3x3]]
    %           y_dot = [q_dot; v_vel_v/w; v_omega_v/w]
    %           --> NB: w_J_d/w = wRv * v_J_d/w = [0[3xl] -wRv 0[3x3]]
    % 6: d_dot_des = - lamda * d
    % 7: d_dot_des = v_g/w - v_v/w_des = - lamda * d
    % 8: - v_v/w_des = - lamda * d - v_g/w = x_d/w_des --> projected on the same ref.frame of the Jacobian
    % 9: y_dot = psinv_J_d/w * x_d/w_des
    %    d_dot = v_g/w + J_d/w * psinv_J_d/w * (- lambda * d - v_g/w)
    % 10: A_d = I[3x3] (because it's an equality task)


    %NOTA: V_g/w = 0 ???


    methods
        function updateReference(obj, robot)
            [ang, lin] = CartError(robot.wTgv , robot.wTv);
            disp(lin);
            obj.xdotbar = - 0.2 * [0; 0; 0; lin];
            % limit the requested velocities...
            obj.xdotbar(1:3) = Saturate(obj.xdotbar(1:3), 0.2);
            obj.xdotbar(4:6) = Saturate(obj.xdotbar(4:6), 0.2);
        end
        function updateJacobian(obj, robot)
            Jt_a  = zeros(6,7);
            wRv = robot.wTv(1:3, 1:3);
            Jt_v = [ (-wRv) zeros(3); zeros(3) zeros(3)];
            obj.J = [Jt_a Jt_v];
        end
        
        function updateActivation(obj, robot)
            obj.A = eye(6);
        end
    end
end