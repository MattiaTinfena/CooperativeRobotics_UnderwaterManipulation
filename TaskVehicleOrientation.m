classdef TaskVehicleOrientation < Task   
    properties

    end

    methods
        function updateReference(obj, robot)
            [ang,~] = CartError(robot.wTgv , robot.wTv); % I compute the cartesian error between two frames projected on w
            disp(ang);
            obj.xdotbar = 0.2 * [ang];
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
            %obj.A = zeros(3);
            obj.A = eye(3);
            [error, ~]  = CartError(robot.wTgv , robot.wTv);
            errorz = error(3)*180/pi;
            obj.A(3,3) = IncreasingBellShapedFunction(0.1,0.2,0,1,abs(errorz));
            obj.A(3,3) = 0;

            %disp(errorz)
            disp(obj.A)
        end
    end
end

% IO HO DUE RICHIESTE DIVERSE: 

% uno in cui voglio che l'orientazione del veicolo sia
% uguale a quella del goal del veicolo -> controllo l'errore su tutti e 3
% gli assi. quindi ho A = eye(1) -> la priorita' e' comunque alta per
% l'orientazione

% dopodiche' controllo l'errore di orientazione tra il frame del viecolo e
% il frame world e vedo che i loro assi z abbiano meno misalignment di 0.2
% -> controllo solo la rotazione tra world e vehicle (wRv) e la mando a 0
% controllandone la priorita'  con la roba della bell shaped function

