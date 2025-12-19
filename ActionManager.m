classdef ActionManager < handle
    properties
        actions = {}      % cell array of actions (each action = stack of tasks)
        action_names = {}
        currentAction = 1 % index of currently active action
        previousAction = 1
        actionChanges = 0
        action_switch_time = {}
    end

    methods
        function addAction(obj, taskStack, name)
            % taskStack: cell array of tasks that define an action
            obj.actions{end+1} = taskStack;
            obj.action_names{end+1} = name;
        end

        function [v_nu, qdot] = computeICAT(obj, robot)
            % Get current action
            actTasks  = obj.actions{obj.currentAction};
            prevTasks = obj.actions{obj.previousAction};

            tasks = actTasks;

            if (obj.actionChanges)
   
                % %update ap
                % ap = {}

                % %when gaussian transitory is ended
                obj.actionChanges = 0;
                disp(tasks);
            end


            % 1. Update references, Jacobians, activations
            for i = 1:length(tasks)
                tasks{i}.updateReference(robot);
                tasks{i}.updateJacobian(robot);
                tasks{i}.updateActivation(robot);
            end

            % 2. Perform ICAT (task-priority inverse kinematics)
            ydotbar = zeros(13,1);
            Qp = eye(13);
            for i = 1:length(tasks)
                [Qp, ydotbar] = iCAT_task(tasks{i}.A, tasks{i}.J, ...
                                           Qp, ydotbar, tasks{i}.xdotbar, ...
                                           1e-4, 0.01, 10);
            end

            % 3. Last task: residual damping
            [~, ydotbar] = iCAT_task(eye(13), eye(13), Qp, ydotbar, zeros(13,1), 1e-4, 0.01, 10);

            % 4. Split velocities for vehicle and arm
            qdot = ydotbar(1:7);
            v_nu = ydotbar(8:13); % projected on the vehicle frame
        end

    function setCurrentAction(obj, actionName)

        found = false;
        obj.previousAction = obj.currentAction;
        obj.actionChanges = 1;
        for i = 1:length(obj.action_names)
            if strcmp(obj.action_names{i}, actionName)
                obj.currentAction = i;
                found = true;
                break; % esci dal ciclo
            end
        end

        if ~found
            error('Action not found');
        end
    end

    end
end