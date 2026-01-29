classdef SimulationLogger < handle
    properties
        t
        q
        q_dot
        eta
        v_nu
        altitude
        hor_att
        orientation = []
        nodule_dist
        nodule_align = []

        xdotbar_task
        activation_task
        priority_task
        product_task

        global_activations
        total_tasks

        robot
        action_mng
        n
        curr_loop

        switch_times = []
        switch_names = {}
        last_action_idx = 0
    end

    methods
        function obj = SimulationLogger(maxLoops, robotModel, actionManager)
            obj.robot = robotModel;
            obj.action_mng = actionManager;
            obj.curr_loop = 0;

            obj.t = zeros(1, maxLoops);
            obj.q = zeros(7, maxLoops);
            obj.q_dot = zeros(7, maxLoops);
            obj.eta = zeros(6, maxLoops);
            obj.v_nu = zeros(6, maxLoops);
            obj.altitude = zeros(1, maxLoops);
            obj.hor_att = zeros(1, maxLoops);
            obj.orientation = zeros(3, maxLoops);
            obj.nodule_dist = zeros(1, maxLoops);
            obj.nodule_align = zeros(3, maxLoops);

            obj.n = length(actionManager.actions);
            l = zeros(1, obj.n);
            for i = 1:obj.n
                l(i) = length(actionManager.actions{i});
            end
            max_tasks = max(l);

            obj.xdotbar_task = cell(obj.n, max_tasks, maxLoops);
            obj.activation_task = cell(obj.n, max_tasks, maxLoops);
            obj.priority_task = cell(obj.n, max_tasks, maxLoops);
            obj.product_task = cell(obj.n, max_tasks, maxLoops);
            obj.total_tasks = length(actionManager.all_task_list);
            obj.global_activations = zeros(obj.total_tasks, maxLoops);

            obj.last_action_idx = 0;
        end

        function update(obj, t, loop)
            obj.curr_loop = loop;
            obj.t(loop) = t;

            current_act = obj.action_mng.current_action;
            if current_act ~= obj.last_action_idx
                obj.switch_times(end+1) = t;
                if ~isempty(obj.action_mng.actions_names)
                    obj.switch_names{end+1} = obj.action_mng.actions_names{current_act};
                else
                    obj.switch_names{end+1} = sprintf("Act %d", current_act);
                end
                obj.last_action_idx = current_act;
            end

            obj.q(:, loop) = obj.robot.q;
            obj.q_dot(:, loop) = obj.robot.q_dot;
            obj.eta(:, loop) = obj.robot.eta;
            obj.v_nu(:, loop) = obj.robot.v_nu;
            obj.altitude(loop) = norm(obj.robot.altitude);

            obj.hor_att(loop) = obj.robot.theta;

            [obj.orientation(:, loop), ~] = CartError(obj.robot.wTgv , obj.robot.wTv);
            [~,lin] = CartError(obj.robot.wTg, obj.robot.wTv);
            obj.nodule_dist(loop) = norm(lin(1:2));

            v_pos = obj.robot.wTv(1:3, 4);
            n_pos = obj.robot.wTg(1:3, 4);
            error_x = n_pos(1) - v_pos(1);
            error_y = n_pos(2) - v_pos(2);
            des_or = atan2(error_y, error_x);
            wRv_des = rotation(0, 0, des_or);
            wTv_des = [wRv_des, v_pos; 0 0 0 1];
            [obj.nodule_align(:, loop), ~] = CartError(wTv_des, obj.robot.wTv);

            for i = 1:obj.n
                tasks = obj.action_mng.actions{i};
                for j = 1:length(tasks)
                    task = tasks{j};

                    if isempty(task.xdotbar)
                        obj.xdotbar_task{i,j,loop} = 0;
                    else
                        obj.xdotbar_task{i,j,loop} = task.xdotbar;
                    end

                    val_A = task.A;
                    val_A_vec = 0;

                    if isempty(val_A)
                        obj.activation_task{i,j,loop} = 0;
                    else
                        if size(val_A, 1) > 1 && size(val_A, 2) > 1
                            val_A_vec = diag(val_A);
                        else
                            val_A_vec = val_A;
                        end
                        obj.activation_task{i,j,loop} = val_A_vec;
                    end

                    val_ap = 0;
                    if isempty(task.ap)
                        obj.priority_task{i,j,loop} = 0;
                    else
                        val_ap = task.ap;
                        obj.priority_task{i,j,loop} = task.ap;
                    end

                    if isempty(task.A) || isempty(task.ap)
                        obj.product_task{i,j,loop} = 0;
                    else
                        obj.product_task{i,j,loop} = val_A_vec * val_ap;
                    end
                end
            end

            for k = 1:obj.total_tasks
                t_ptr = obj.action_mng.all_task_list{k};

                if isempty(t_ptr.A)
                    val_A_scalar = 0;
                elseif ismatrix(t_ptr.A) && ~isscalar(t_ptr.A)
                    val_A_scalar = max(diag(t_ptr.A));
                else
                    val_A_scalar = t_ptr.A;
                end

                if isempty(t_ptr.ap)
                    val_ap = 0;
                else
                    val_ap = t_ptr.ap;
                end

                obj.global_activations(k, loop) = val_A_scalar * val_ap;
            end
        end

        function plotAll(obj)
            % --- STYLE SETTINGS ---
            LW = 3;       % Line Width
            FS = 20;        % Font Size (Axes, Legends)
            FS_Title = 23;  % Font Size (Titles)
            % ----------------------

            t_plot = obj.t(1:obj.curr_loop);

            % --- Plot 1: Arm State ---
            figure(1);
            subplot(2,1,1);
            plot(t_plot, obj.q(:, 1:obj.curr_loop), 'LineWidth', LW);
            title('Arm Joint Positions', 'FontSize', FS_Title);
            grid on; set(gca, 'FontSize', FS);
            legend('q_1','q_2','q_3','q_4','q_5','q_6','q_7', 'FontSize', FS);

            subplot(2,1,2);
            plot(t_plot, obj.q_dot(:, 1:obj.curr_loop), 'LineWidth', LW);
            title('Arm Joint Velocities', 'FontSize', FS_Title);
            grid on; set(gca, 'FontSize', FS);
            legend('qd_1','qd_2','qd_3','qd_4','qd_5','qd_6','qd_7', 'FontSize', FS);
            sgtitle('Arm Motion', 'FontSize', FS_Title + 2, 'FontWeight', 'bold');

            % --- Plot 2: Vehicle State ---
            figure(2);
            subplot(3,1,1);
            plot(t_plot, obj.eta(:, 1:obj.curr_loop), 'LineWidth', LW);
            title('Vehicle Pose', 'FontSize', FS_Title);
            grid on; set(gca, 'FontSize', FS);
            legend('x','y','z','r','p','y', 'FontSize', FS);

            subplot(3,1,2);
            plot(t_plot, obj.v_nu(:, 1:obj.curr_loop), 'LineWidth', LW);
            title('Vehicle Velocities', 'FontSize', FS_Title);
            grid on; set(gca, 'FontSize', FS);
            legend('u','v','w','p','q','r', 'FontSize', FS);
            sgtitle('Vehicle Motion', 'FontSize', FS_Title + 2, 'FontWeight', 'bold');

            % --- Plot 3: All tasks global activation ---
            figure(20); clf; hold on;
            for k = 1:obj.total_tasks
                plot(t_plot, obj.global_activations(k, 1:obj.curr_loop), 'LineWidth', LW);
            end

            % Draw Action Switch Lines
            y_limits = ylim; if y_limits(2) < 1.1, y_limits(2) = 1.1; end
            ylim([-0.1, y_limits(2)]);

            for s = 1:length(obj.switch_times)
                xline(obj.switch_times(s), '--k', 'LineWidth', LW, 'Alpha', 0.6);
                text(obj.switch_times(s), y_limits(1) + 0.05, ['  ' obj.switch_names{s}], ...
                    'Rotation', 90, 'VerticalAlignment', 'bottom', ...
                    'HorizontalAlignment', 'left', 'FontSize', FS - 2, ...
                    'FontWeight', 'bold', 'Interpreter', 'none');
            end

            hold off; grid on;
            xlabel('Time [s]', 'FontSize', FS);
            ylabel('Activation (A * ap)', 'FontSize', FS);
            title('Global Task Activations', 'FontSize', FS_Title);
            set(gca, 'FontSize', FS);

            if ~isempty(obj.action_mng.all_task_names)
                legend(obj.action_mng.all_task_names, 'Interpreter', 'none', 'Location', 'eastoutside', 'FontSize', FS, 'HandleVisibility', 'off');
            else
                legend(arrayfun(@(x) sprintf('Task %d', x), 1:obj.total_tasks, 'UniformOutput', false), 'FontSize', FS);
            end

            metrics_data = { ...
                obj.altitude(1:obj.curr_loop), ...
                obj.hor_att(1:obj.curr_loop), ...
                obj.orientation(:, 1:obj.curr_loop), ...
                obj.nodule_dist(1:obj.curr_loop), ...
                obj.nodule_align(:, 1:obj.curr_loop) ...
                };
            metrics_titles = { ...
                'Altitude', ...
                'Theta', ...
                'Orientation misalignment', ...
                'Nodule distance', ...
                'Nodule misalignment' ...
                };
            metrics_ylabels = { ...
                'Alt [m]', ...
                'Angle [rad]', ...
                'Angle [rad]', ...
                'Distance [m]', ...
                'Angle [rad]' ...
                };
            metrics_legends = { ...
                {}, ...
                {}, ...
                {'roll','pitch','yaw'}, ...
                {}, ...
                {'roll','pitch','yaw'} ...
                };

            for k = 1:obj.total_tasks

                target_task = obj.action_mng.all_task_list{k};
                act_idx = -1; tsk_idx = -1;
                found = false;
                for a = 1:obj.n
                    act_tasks = obj.action_mng.actions{a};
                    for t = 1:length(act_tasks)
                        if act_tasks{t} == target_task
                            act_idx = a; tsk_idx = t; found = true; break;
                        end
                    end
                    if found, break; end
                end

                if ~found
                    fprintf('Task %d skipped (not in action stacks).\n', k);
                    continue;
                end

                raw_xdot = squeeze(obj.xdotbar_task(act_idx, tsk_idx, 1:obj.curr_loop));
                raw_A    = squeeze(obj.activation_task(act_idx, tsk_idx, 1:obj.curr_loop));
                raw_ap   = squeeze(obj.priority_task(act_idx, tsk_idx, 1:obj.curr_loop));
                raw_prod = squeeze(obj.product_task(act_idx, tsk_idx, 1:obj.curr_loop));

                data_xdot = obj.fillWithZeros(raw_xdot);
                data_A    = obj.fillWithZeros(raw_A);
                data_ap   = obj.fillWithZeros(raw_ap);
                data_prod = obj.fillWithZeros(raw_prod);

                f = figure(100 + k);
                clf;
                task_name = string(obj.action_mng.all_task_names(k));
                f.Name = sprintf("Task Analysis: %s", task_name);

                tg = uitabgroup(f);

                for m = 1:length(metrics_data)

                    tab = uitab(tg, 'Title', metrics_titles{m});

                    ax1 = subplot(3,1,1, 'Parent', tab);

                    % --- PLOT 1: Velocity  ---
                    plot(ax1, t_plot, data_xdot', 'LineWidth', LW);
                    ylabel(ax1, '$\dot{\bar{x}}$', 'Interpreter', 'latex', 'FontSize', FS+2);
                    grid(ax1, 'on');
                    title(ax1, ['Reference Velocity - ' char(task_name)], 'Interpreter', 'none', 'FontSize', FS_Title);
                    set(ax1, 'FontSize', FS);
                    labels = arrayfun(@(x) sprintf('xd_{%d}', x), 1:size(data_xdot,1), 'UniformOutput', false);
                    legend(ax1, labels, 'FontSize', FS, 'Location', 'eastoutside');
                    for s = 1:length(obj.switch_times)
                        xline(ax1, obj.switch_times(s), ':k', 'Alpha', 0.3, 'LineWidth', LW, 'HandleVisibility', 'off');
                    end
                    % --- PLOT 2: Activation ---
                    ax2 = subplot(3,1,2, 'Parent', tab);
                    hold(ax2, 'on');
                    h_ap = plot(ax2, t_plot, data_ap', '--k', 'LineWidth', LW);
                    h_A = plot(ax2, t_plot, data_A', '-b', 'LineWidth', LW/2 + 0.5);
                    h_prod = plot(ax2, t_plot, data_prod', '-.r', 'LineWidth', LW);

                    for s = 1:length(obj.switch_times)
                        xline(ax2, obj.switch_times(s), ':k', 'Alpha', 0.3, 'LineWidth', LW, 'HandleVisibility', 'off');
                    end

                    hold(ax2, 'off');
                    grid(ax2, 'on');
                    ylabel(ax2, 'Activation', 'FontSize', FS);
                    title(ax2, 'Activation Levels', 'FontSize', FS_Title);
                    set(ax2, 'FontSize', FS);
                    if ~isempty(h_ap) && ~isempty(h_A) && ~isempty(h_prod)
                        legend(ax2, [h_ap(1), h_A(1), h_prod(1)], ...
                            {'$a_p$', '$A$', '$Total$'}, ...
                            'Interpreter', 'latex', 'Location', 'eastoutside', 'FontSize', FS);
                    end

                    % --- PLOT 3: Specific metrics ---
                    ax3 = subplot(3,1,3, 'Parent', tab);
                    curr_metric = metrics_data{m};

                    if isvector(curr_metric) && size(curr_metric,1) == 1
                        plot(ax3, t_plot, curr_metric, 'LineWidth', LW, 'Color', [0 0.5 0]);
                    else
                        plot(ax3, t_plot, curr_metric, 'LineWidth', LW);
                    end

                    for s = 1:length(obj.switch_times)
                        xline(ax3, obj.switch_times(s), ':k', 'Alpha', 0.3, 'LineWidth', LW);
                    end

                    grid(ax3, 'on');
                    title(ax3, metrics_titles{m}, 'FontSize', FS_Title);
                    ylabel(ax3, metrics_ylabels{m}, 'FontSize', FS);
                    xlabel(ax3, 'Time [s]', 'FontSize', FS);
                    set(ax3, 'FontSize', FS);

                    if ~isempty(metrics_legends{m})
                        legend(ax3, metrics_legends{m}, 'FontSize', FS, 'Location', 'eastoutside');
                    else
                        lgd = legend(ax3, ' ', 'Location', 'eastoutside');
                        lgd.Visible = 'on';
                        lgd.Color = 'none';
                        lgd.EdgeColor = 'none';
                        lgd.TextColor = 'none';
                        lgd.HandleVisibility = 'off';
                    end
                end
            end
        end

        function data_mat = fillWithZeros(~, cell_data)
            dim = 1;
            for k = 1:length(cell_data)
                val = cell_data{k};
                if ~isempty(val) && ~isscalar(val)
                    dim = size(val, 1);
                    break;
                end
            end
            for k = 1:length(cell_data)
                if isempty(cell_data{k}) || (isscalar(cell_data{k}) && dim > 1)
                    cell_data{k} = zeros(dim, 1);
                end
            end
            data_mat = cell2mat(cell_data');
            if size(data_mat, 2) ~= length(cell_data)
                data_mat = data_mat';
            end
        end
    end
end