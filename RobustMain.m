% Add paths
addpath('./simulation_scripts');
addpath('./tools');
addpath('./icat');
addpath('./robust_robot');
clc; clear; close all;
% Simulation parameters
dt       = 0.005;
endTime  = 150;
% Initialize robot model and simulator
robotModel = UvmsModel();          
sim = UvmsSim(dt, robotModel, endTime);
% Initialize Unity interface
unity = UnityInterface("127.0.0.1");

% Define task
task_vehicle_position = TaskVehiclePosition();       
task_tool    = TaskTool();
task_vehicle_hor_att = TaskVehicleHorAtt();
task_vehicle_altitude = TaskVehicleAltitude();
task_zero_altitude = TaskZeroAltitude();

safe_navigation = ["TVA", "TVHA", "TVP"];
landing = ["TZA", "TVHA", "TVP"];

task_list = {task_vehicle_position, task_tool, task_vehicle_hor_att, task_vehicle_altitude, task_zero_altitude};
task_list_name = ["TVP", "TT", "TVHA", "TVA", "TZA"];

% Define actions and add to ActionManager
actionManager = ActionManager();
actionManager.setTaskList(task_list, task_list_name);
actionManager.addAction(safe_navigation, "SN");  % action 1
actionManager.addAction(landing, "L");  % action 2

actionManager.setCurrentAction("SN", sim.time);

% Define desired positions and orientations (world frame)
w_arm_goal_position = [12.2025, 37.3748, -39.8860]';
w_arm_goal_orientation = [0, pi, pi/2];
w_vehicle_goal_position = [50   -12.5  -33]';
w_vehicle_goal_orientation = [0, 0, 0];

% Set goals in the robot model
robotModel.setGoal(w_arm_goal_position, w_arm_goal_orientation, w_vehicle_goal_position, w_vehicle_goal_orientation);

% Initialize the logger
% logger = SimulationLogger(ceil(endTime/dt)+1, robotModel, landing);

% Main simulation loop
for step = 1:sim.maxSteps
    % 1. Receive altitude from Unity
    robotModel.altitude = unity.receiveAltitude(robotModel);

    % 2. Compute control commands for current action
    [v_nu, q_dot] = actionManager.computeICAT(robotModel, sim.time);

    % 3. Step the simulator (integrate velocities)
    sim.step(v_nu, q_dot);

    % 4. Send updated state to Unity
    unity.send(robotModel);

    % 5. Logging
    % logger.update(sim.time, sim.loopCounter);

    % 6. Optional debug prints

    [~,lin] = CartError(robotModel.wTgv , robotModel.wTv); % I compute the cartesian error between two frames projected on w
    if norm(lin) < 0.1 && actionManager.current_action == 1 %valuta se mettere la norma solo di x e y (nel caso in cui z non sia raggiungibile)
        actionManager.setCurrentAction("L", sim.time);
    end
    
    if mod(sim.loopCounter, round(1 / sim.dt)) == 0
        fprintf('t = %.2f s\n', sim.time);
        fprintf('alt = %.2f m\n', robotModel.altitude);
    end
    % 7. Optional real-time slowdown
    SlowdownToRealtime(dt);
end

% Display plots
% logger.plotAll();

% Clean up Unity interface
delete(unity);