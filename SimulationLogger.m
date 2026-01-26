classdef SimulationLogger < handle
    properties
        t            % time vector
        q            % joint positions (arm)
        q_dot        % joint velocities (arm)
        eta          % vehicle pose
        v_nu         % vehicle velocities
        xdotbar_task % reference velocities for tasks (cell array)
        robot        % robot model
        action_mng   % action manager
        n            % number of actions
    end

    methods
        function obj = SimulationLogger(maxLoops, robotModel, actionManager)
            obj.robot = robotModel;
            obj.action_mng = actionManager;

            obj.t = zeros(1, maxLoops);
            obj.q = zeros(7, maxLoops);
            obj.q_dot = zeros(7, maxLoops);
            obj.eta = zeros(6, maxLoops);
            obj.v_nu = zeros(6, maxLoops);
            obj.n = length(actionManager.actions);
            l = zeros(1, obj.n);
            for i = 1:obj.n
                l(i) = length(actionManager.actions{i});
            end

            obj.xdotbar_task = cell(obj.n, max(l), maxLoops);
        end

        function update(obj, t, loop)
            % Store robot state
            obj.t(loop) = t;
            obj.q(:, loop) = obj.robot.q;
            obj.q_dot(:, loop) = obj.robot.q_dot;
            obj.eta(:, loop) = obj.robot.eta;
            obj.v_nu(:, loop) = obj.robot.v_nu;

            % Store task reference velocities
            for i = 1:obj.n
                tasks = obj.action_mng.actions{i};
                for j = 1:length(tasks)
                    obj.xdotbar_task{i,j,loop} = tasks{j}.xdotbar;
                end
            end
        end

        function plotAll(obj, action_idx, task_indices)
            % Plot 1: Arm State
            figure(1);
            subplot(2,1,1);
            title('Arm Joint Positions');
            plot(obj.t, obj.q, 'LineWidth', 1.5);
            legend('q_1','q_2','q_3','q_4','q_5','q_6','q_7');
            grid on;

            subplot(2,1,2);
            title('Arm Joint Velocities');
            plot(obj.t, obj.q_dot, 'LineWidth', 1.5);
            legend('qd_1','qd_2','qd_3','qd_4','qd_5','qd_6','qd_7');
            grid on;

            % Plot 2: Vehicle State
            figure(2);
            title('Vehicle Motion');
            subplot(2,1,1);
            title('Vehicle Pose');
            plot(obj.t, obj.eta, 'LineWidth', 1.5);
            legend('x','y','z','r','p','y');
            grid on;

            subplot(2,1,2);
            title('Vehicle Velocities');
            plot(obj.t, obj.v_nu, 'LineWidth', 1.5);
            legend('u','v','w','p','q','r');
            grid on;

            % Plot 3: Specific Tasks from an Action
            if nargin > 1 && ~isempty(action_idx) && ~isempty(task_indices)
                figure(3);
                s = squeeze(obj.xdotbar_task(action_idx, :, :));
                nt = length(task_indices);
                if action_idx <= length(obj.action_mng.actions_names)
                    act_name = obj.action_mng.actions_names{action_idx};
                else
                    act_name = num2str(action_idx);
                end
                sgtitle(['Action: ', char(act_name)]);

                for i = 1:nt
                    subplot(nt, 1, i);
                    t_idx = task_indices(i);
                    data = cell2mat(s(t_idx, :));
                    if isempty(data)
                        title(['Task ' num2str(t_idx) ' (No Data)']);
                        continue;
                    end

                    len_data = size(data, 2);
                    plot(obj.t(1:len_data), data', '.-', 'LineWidth', 1);
                    grid on;
                    try
                        curr_task = obj.action_mng.actions{action_idx}{t_idx};
                        t_name = class(curr_task);
                        title(['Task ', num2str(t_idx), ': ', t_name]);
                        if size(data,1) == 6
                            legend('v_x','v_y','v_z','w_x','w_y','w_z');
                        end
                    catch
                        title(['Task ', num2str(t_idx)]);
                    end
                end
                xlabel('Time [s]');
            end
        end
    end
end