classdef TaskVehicle < Task   
    properties

    end

    % 1: x(c) = d = (Ogoal - Ovehicle)
    % 2: I want my d to go to 0
    % 3: Identify the time behaviour of 
    
    %   d -> d_dot = - v_v/w

    % 4: - d_dot = v_v/w (independent from vehicle = dep. from vehicle)
    % 5: I want to express the equation in matrix form:

    %       d_dot = v_J_d/w * y_dot, where 

    %           v_J_d/w = [-I[3x3] 0[3x3]]

    %           y_dot = [v_vel_v/w; v_omega_v/w]

    %           --> NB: w_J_d/w = wRv * v_J_d/w = [-wRv 0[3x3]]

    % 6: d_dot_des = - lamda * d

    % 7: d_dot_des = - v_v/w_des = - lamda * d

    % 8: - v_v/w_des = - lamda * d = x_d/w_des --> projected on the same ref.frame of the Jacobian

    % 9: y_dot = psinv_J_d/w * x_d/w_des

    %    d_dot = J_d/w * psinv_J_d/w * (- lambda * d)

    % 10: A_d = I[3x3] (because it's an equality task)


    % NB: V_g/w = 0 ???
    % xdotbar = x_d/w_des, lin = d

    methods
        function updateReference(obj, robot)
            [~,lin] = CartError(robot.wTgv , robot.wTv); % I compute the cartesian error between two frames projected on w
            %disp(lin);
            obj.xdotbar = - 0.2 * [lin];
            % limit the requested velocities...
            obj.xdotbar = Saturate(obj.xdotbar, 0.2);
        end
        function updateJacobian(obj, robot)
            Jt_a  = zeros(3,7);
            wRv = robot.wTv(1:3, 1:3);
            Jt_v = [(-wRv) zeros(3)];
            obj.J = [Jt_a Jt_v];
        end
        
        function updateActivation(obj, robot)
            obj.A = eye(3);
        end
    end
end