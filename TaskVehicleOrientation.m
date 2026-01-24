classdef TaskVehicleOrientation < Task
    properties

    end

    methods
        function updateReference(obj, robot)

            p_vehicle = robot.wTv(1:3, 4);

            % 2. Recupera la posizione del nodulo (target del braccio)
            % Assumiamo che robot.goalPosition contenga le coordinate del nodulo
            p_nodule = robot.wTg(1:3, 4);

            % 3. Calcola il vettore direzione sul piano orizzontale
            delta_x = p_nodule(1) - p_vehicle(1);
            delta_y = p_nodule(2) - p_vehicle(2);

            % 4. Calcola lo Yaw desiderato (allineamento asse X verso il nodulo)
            yaw_des = atan2(delta_y, delta_x);

            % 5. Costruisci la matrice di rotazione desiderata
            % Roll = 0, Pitch = 0 (per atterraggio piatto), Yaw = yaw_des
            % Usiamo la funzione rotation(r, p, y) che presumibilmente hai nei tools
            wR_des = rotation(0, 0, yaw_des);

            % Costruiamo una trasformazione target fittizia (la posizione non importa per l'errore angolare)
            wT_des_att = [wR_des, p_vehicle; 0 0 0 1];

            % 6. Calcola l'errore angolare usando CartError
            % CartError restituisce [errore_angolare, errore_lineare]
            [ang, ~] = CartError(wT_des_att, robot.wTv);

            % 7. Legge di controllo
            % Usiamo la stessa convenzione del tuo TaskVehiclePosition (-lambda * error)
            obj.xdotbar = - 0.2 * ang;

            % Saturazione
            obj.xdotbar = Saturate(obj.xdotbar, 0.2);
        end
        function updateJacobian(obj, robot)
            % La Jacobiana deve mettere in relazione [q_dot; v_v; w_v] con la velocità angolare del mondo w_w.
            % Relazione: w_w = wRv * w_v

            Jt_a  = zeros(3,7); % Il braccio non influenza l'orientamento della base
            wRv = robot.wTv(1:3, 1:3);

            % La parte del veicolo è [Zeros(3x3) | -wRv]
            % Nota: Usiamo -wRv per coerenza con la legge di controllo definita come -lambda*e
            % (Se CartError è definito come T_des - T_curr, allora serve il segno meno nella Jacobiana o nel guadagno)
            Jt_v = [zeros(3) (-wRv)];

            obj.J = [Jt_a Jt_v];
        end

        function updateActivation(obj, robot)
            obj.A = eye(3);
        end
    end
end