% Add paths
addpath('./simulation_scripts');
addpath('./tools');
addpath('./icat');
addpath('./robust_robot');
clc; clear; close all;
% Simulation parameters
dt       = 0.005;
endTime  = 80;
% Initialize robot model and simulator
robotModel = UvmsModel();
sim = UvmsSim(dt, robotModel, endTime);
% Initialize Unity interface
unity = UnityInterface("127.0.0.1");

% Define task
task_vehicle_position = TaskVehiclePosition();
task_tool = TaskTool();
task_vehicle_hor_att = TaskVehicleHorAtt();
task_vehicle_altitude = TaskVehicleAltitude();
task_vehicle_orientation = TaskVehicleOrientation();
task_align_to_nodule = TaskAlignToNodule();
task_zero_altitude = TaskZeroAltitude();
task_not_moving = TaskNotMoving();
task_max_nodule_dist = TaskMaxNoduleDist();

safe_waypoint_navigation_action = ["TVA", "TVHA", "TVP", "TVO"];
positioning_action = ["TVHA", "TMD", "TATN"];
landing_action = ["TVHA", "TATN","TZA"];
fixed_based_manipulation_action = ["TNM", "TT"];

task_list = {task_vehicle_position, task_tool, task_vehicle_hor_att, task_vehicle_altitude, task_zero_altitude, task_vehicle_orientation, task_not_moving, task_max_nodule_dist, task_align_to_nodule};
task_list_name = ["TVP", "TT", "TVHA", "TVA", "TZA", "TVO", "TNM", "TMD", "TATN"];

% Define actions and add to ActionManager
actionManager = ActionManager();
actionManager.setTaskList(task_list, task_list_name);
actionManager.addAction(safe_waypoint_navigation_action, "SN");  % action 1
actionManager.addAction(positioning_action, "P");  % action 2
actionManager.addAction(landing_action, "L");  % action 3
actionManager.addAction(fixed_based_manipulation_action, "M") % action 4

actionManager.setCurrentAction("SN", sim.time);

initial_time = sim.time;
% Define desired positions and orientations (world frame)
w_arm_goal_position = [12.2025, 37.3748, -39.8860]';
w_arm_goal_orientation = [0, pi, pi/2];
w_vehicle_goal_position = [10.5 37.5 -38]';
w_vehicle_goal_orientation = [0, -0.06, 0.5];

% w_vehicle_goal_position = [50 11 -33]'; % to stress the minimum altitude task
% w_vehicle_goal_position = [10.5 40 -38]'; % to stress the distance and the alignment from nodule

% Set goals in the robot model
robotModel.setGoal(w_arm_goal_position, w_arm_goal_orientation, w_vehicle_goal_position, w_vehicle_goal_orientation);

% Initialize the logger
logger = SimulationLogger(ceil(endTime/dt)+1, robotModel, actionManager);

% Main simulation loop
for step = 1:sim.maxSteps
    % 1. Receive altitude from Unity
    robotModel.altitude = unity.receiveAltitude(robotModel);

    prev_pos = robotModel.wTv;

    % 2. Compute control commands for current action
    [v_nu, q_dot] = actionManager.computeICAT(robotModel, sim.time);

    % 4. Step the simulator (integrate velocities)
    sim.step(v_nu, q_dot);

    % 3. Action switching

    act_pos = robotModel.wTv;

    [ang_dist,lin_dist] = CartError(act_pos , prev_pos); % I compute the cartesian error between two frames projected on w

    % Simple watchdog to evaluate if the robot is stacked beacuse the goal is not reachable
    if norm(ang_dist) > 0.0002 || norm(lin_dist) > 0.0002
        initial_time = sim.time;
    end

    [ang_error,lin_error] = CartError(robotModel.wTgv , robotModel.wTv); % I compute the cartesian error between two frames projected on w
    [~,nodule_dist] = CartError(robotModel.wTg, robotModel.wTv);

    delta_time = sim.time - initial_time;

    if ((norm(lin_error) < 0.1 && norm(ang_error) < 0.1) || delta_time > 7) && actionManager.current_action == 1
        actionManager.setCurrentAction("P", sim.time);
        initial_time = sim.time;

    elseif ((norm(nodule_dist(1:2)) < 1.7 && norm(task_align_to_nodule.nodule_misalignment) < 0.1 ) || delta_time > 7) && actionManager.current_action == 2
        actionManager.setCurrentAction("L", sim.time);
        initial_time = sim.time;
    end

    if norm(robotModel.altitude) < 0.02 && actionManager.current_action == 3
        actionManager.setCurrentAction("M", sim.time);
    end

    % 5. Send updated state to Unity
    unity.send(robotModel);

    % 6. Logging
    logger.update(sim.time, sim.loopCounter);

    % 7. Optional debug prints

    if mod(sim.loopCounter, round(1 / sim.dt)) == 0
        fprintf('t = %.2f s\n', sim.time);
        fprintf('alt = %.2f m\n', robotModel.altitude);
        disp(delta_time);
    end

    % 8. Optional real-time slowdown
    SlowdownToRealtime(dt);
end

% Display plots
logger.plotAll();

% Clean up Unity interface
delete(unity);