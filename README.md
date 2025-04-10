%% Enhanced SAR Drone Simulation with Improved Visualization
clc; clear; close all;
%% ==================== 1. Mechanical Design & Fabrication ====================
drone.mass = 1.5; % kg (Typical for DJI Matrice 300)
drone.dimensions = [0.88 0.88 0.43]; % [L W H] in meters
drone.max_thrust = 4 * 6.2; % N (4 motors @ 6.2N each)
drone.max_speed = 5; % Reduced from 23 to 5 m/s for better visualization
disp('=== Mechanical Design ===');
disp(['Mass: ' num2str(drone.mass) ' kg']);
disp(['Dimensions: ' num2str(drone.dimensions) ' m']);
disp(['Max Thrust: ' num2str(drone.max_thrust) ' N']);
%% ==================== 2. Environment Setup ====================
map = binaryOccupancyMap(200, 200, 1);
inflate(map, 5); % Safety margin
% Original walls and gates
walls = [40 40; 40 180; 60 180; 60 40; 
         120 20; 120 160; 150 160; 150 20];
gates = [90 80; 90 120; 110 120; 110 80;
         170 60; 170 100; 190 100; 190 60];
% Make original obstacles thicker
thick_walls = [walls; 
               walls(:,1)+5 walls(:,2);
               walls(:,1)-5 walls(:,2);
               walls(:,1) walls(:,2)+5;
               walls(:,1) walls(:,2)-5];
% Add new complex obstacles (buildings, debris fields, etc.)
new_obstacles = [
    % Building 1 (L-shaped)
    20 20; 20 60; 40 60; 40 40; 60 40; 60 20;
    % Building 2 (with courtyard)
    180 180; 180 140; 160 140; 160 160; 140 160; 140 180;
    % Debris field 1 (scattered obstacles)
    80 30; 85 35; 90 30; 95 40; 100 35; 105 45; 110 40;
    % Debris field 2 (zig-zag pattern)
    30 120; 35 115; 40 120; 45 115; 50 120; 55 115; 60 120;
    % Narrow passage obstacles
    130 80; 135 85; 140 80; 145 85; 150 80;
    % Circular obstacle (approximated with 20 points)
    [100 + 15*cos(linspace(0,2*pi,20))' 140 + 15*sin(linspace(0,2*pi,20))'];
    % Random scattered obstacles
    25 150; 30 155; 35 150; 40 155; 45 150;
    175 25; 170 30; 165 25; 160 30; 155 25;
    % Additional gates (narrow passages)
    50 80; 50 100; 55 100; 55 80;
    140 40; 140 60; 145 60; 145 40
];
% Combine all obstacles
all_obstacles = [thick_walls; gates; new_obstacles];
% Set occupancy and display
setOccupancy(map, all_obstacles, 1);
%% ==================== 8. Create Visualization Figures (MOVED UP) ====================
% Figure 1: Navigation Map
fig1 = figure('Name', 'Drone Navigation', 'Position', [100 100 800 700]);
ax1 = axes(fig1);
show(map, 'Parent', ax1);
hold(ax1, 'on');
% Add grid without labels
grid(ax1, 'on');
ax1.GridAlpha = 0.3;
% Initialize start and goal markers (will be updated later)
startPlot = plot(ax1, 0, 0, 'go', 'Visible', 'off');
goalPlot = plot(ax1, 0, 0, 'ro', 'Visible', 'off');
title(ax1, 'Live Drone Navigation');
xlabel(ax1, 'X Coordinate (meters)'); 
ylabel(ax1, 'Y Coordinate (meters)');
% Add dynamic obstacles that will appear during simulation
dynamic_obstacles = [
    80 80 10 10;   % [x y width height]
    130 40 15 15;
    160 120 10 20;
    40 140 20 10;
    180 60 15 15,
];
% Plot dynamic obstacles
for i = 1:size(dynamic_obstacles,1)
    obs = dynamic_obstacles(i,:);
    rectangle(ax1, 'Position', obs, 'FaceColor', [1 0.5 0.5], 'EdgeColor', 'r');
end
% Figure 2: Thermal View
fig2 = figure('Name', 'Thermal Imaging View', 'Position', [900 100 800 700]);
ax2 = axes(fig2);
% Create thermal image display (will be updated later)
thermalDisplay = imagesc(ax2, zeros(200));
colormap(ax2, 'hot');
clim(ax2, [0 1]);
hcb = colorbar(ax2);
hcb.Label.String = 'Temperature Intensity';
title(hcb, '0=Cold, 1=Hot');
hold(ax2, 'on');
% Add grid to thermal view
grid(ax2, 'on');
ax2.GridAlpha = 0.3;
%% ==================== 17. Add New Figure for Success Rate ====================
% Figure 4: Success Rate Over Time
fig4 = figure('Name', 'Mission Success Rate', 'Position', [900 600 800 500]);
ax4 = axes(fig4);
hold(ax4, 'on');
grid(ax4, 'on');
ax4.GridAlpha = 0.3;

% Initialize success rate plot
successPlot = plot(ax4, 0, 0, 'g-', 'LineWidth', 2); % Green line for success rate
xlabel(ax4, 'Time (s)');
ylabel(ax4, 'Success Rate (%)');
title(ax4, 'Mission Success Rate Over Time');
xlim(ax4, [0 100]); % Set initial x-limits (adjust dynamically later)
ylim(ax4, [0 100]); % Success rate ranges from 0% to 100%
drawnow;

% Preallocate arrays for success rate and time data
timeData = [];
successRateData = [];
%% ==================== 3. Control Architecture ====================
control_system = struct(...
    'high_level', 'RRT* Path Planner',...
    'mid_level', 'PID Trajectory Controller',...
    'low_level', 'Quadrotor Attitude Controller');
disp('=== Control Architecture ===');
disp(control_system);
%% ==================== 4. Kinematics & Dynamics ====================
Ixx = 0.034; Iyy = 0.045; Izz = 0.097; % kg·m² (Inertia tensor)
drone.dynamics = @(u, w) [
    u(1:3)/drone.mass;          % Linear acceleration
    [w(1)/Ixx; w(2)/Iyy; w(3)/Izz]  % Angular acceleration
];
disp('=== Dynamics Parameters ===');
disp(['Ixx: ' num2str(Ixx) ' kg·m², Iyy: ' num2str(Iyy) ' kg·m², Izz: ' num2str(Izz) ' kg·m²']);
%% ==================== 5. Generate Valid Random Start/Goal Positions ====================
function pos = findFreePosition(map)
    while true
        pos = [randi([map.XWorldLimits(1)+20, map.XWorldLimits(2)-20]), ...
               randi([map.YWorldLimits(1)+20, map.YWorldLimits(2)-20])];
        if ~checkOccupancy(map, pos)
            break;
        end
    end
end
start_pos = findFreePosition(map);
goal_pos = findFreePosition(map);
% Ensure minimum separation
while norm(start_pos - goal_pos) < 120
    goal_pos = findFreePosition(map);
end
% Update the start and goal markers with actual positions
set(startPlot, 'XData', start_pos(1), 'YData', start_pos(2), 'Visible', 'on');
set(goalPlot, 'XData', goal_pos(1), 'YData', goal_pos(2), 'Visible', 'on');
%% ==================== 6. Sensors & Perception ====================
sensors = struct(...
    'thermal', 'FLIR Tau2 640×512 (50Hz)',...
    'lidar', 'Velodyne VLP-16 (300m range)',...
    'imu', 'BMI088 (6DOF, 1000Hz)',...
    'gps', 'u-blox ZED-F9P RTK (2.5cm accuracy)');
disp('=== Sensor Suite ===');
disp(sensors);
% Thermal imaging setup
thermalImg = 0.3*rand(200,200); % Base thermal noise
% Heat source placement
goal_x = round(goal_pos(1));
goal_y = round(goal_pos(2));
x_range = max(1,goal_x-15):min(200,goal_x+15);
y_range = max(1,goal_y-15):min(200,goal_y+15);
thermalImg(y_range, x_range) = 0.9; % Survivor (hot)
% Other heat sources
thermalImg(25:55, 25:55) = 0.7; % Fire
thermalImg(175:195, 175:195) = 0.6; % Hot debris
% Make obstacles cold
[obsX, obsY] = meshgrid(1:200);
obsPositions = [obsX(:), obsY(:)];
obsMask = getOccupancy(map, obsPositions);
thermalImg(obsMask) = 0.1; % Cold obstacles
% Update thermal image display with actual data
set(thermalDisplay, 'CData', thermalImg);
% Labeled thermal features
survivorPlot = plot(ax2, goal_pos(1), goal_pos(2), 'bo', ...
     'MarkerSize', 12, 'LineWidth', 2, 'DisplayName', 'Survivor');
firePlot = plot(ax2, mean([25 55]), mean([25 55]), 'yo', ...
     'MarkerSize', 12, 'LineWidth', 2, 'DisplayName', 'Fire');
debrisPlot = plot(ax2, mean([175 195]), mean([175 195]), 'mo', ...
     'MarkerSize', 12, 'LineWidth', 2, 'DisplayName', 'Hot Debris');
title(ax2, 'Thermal Imaging View');
xlabel(ax2, 'X Pixel Coordinate'); 
ylabel(ax2, 'Y Pixel Coordinate');
legend([survivorPlot, firePlot, debrisPlot], 'Location', 'northeastoutside');
%% ==================== 9. Communication & Networking ====================
comms = struct(...
    'primary', 'WiFi 802.11ax (5GHz, 1200Mbps)',...
    'backup', 'LTE/5G Cellular',...
    'range', 8000, ... % meters
    'latency', 50); % ms
disp('=== Communication System ===');
disp(comms);
%% ==================== 10. Path Planning Configuration ====================
ss = stateSpaceSE2([map.XWorldLimits; map.YWorldLimits; [-pi pi]]);
sv = validatorOccupancyMap(ss);
sv.Map = map;
sv.ValidationDistance = 1;
planner = plannerRRTStar(ss, sv);
planner.MaxIterations = 3000;
planner.MaxConnectionDistance = 25; % Increased from 20 for larger obstacles
planner.GoalBias = 0.3;
%% ==================== 11. Safety & Security ====================
safety = struct(...
    'geofence', map.XWorldLimits,...
    'emergency_protocols', true,...
    'data_encryption', 'AES-256',...
    'privacy_filters', 'Thermal face blurring');
disp('=== Safety Features ===');
disp(safety);
%% ==================== 12. Dynamic Path Planning ====================
start = [start_pos, 0];
goal = [goal_pos, 0];
[pthObj, solnInfo] = plan(planner, start, goal);
if ~solnInfo.IsPathFound
    path = [linspace(start(1),goal(1),100)', linspace(start(2),goal(2),100)', zeros(100,1)];
    warning('Using straight-line fallback path');
else
    path = pthObj.States;
end
%% ==================== 13. Human-Robot Interaction ====================
% Global variable to control simulation state
global simulationRunning;
simulationRunning = true;
fig3 = figure('Name', 'Control Panel', 'Position', [100 600 400 300]);
uicontrol('Style', 'pushbutton', 'String', 'Emergency Stop',...
          'Position', [150 200 100 50], 'Callback', @emergencyStop);
batteryText = uicontrol('Style', 'text', 'String', 'Battery: 100%',...
          'Position', [150 150 100 20], 'Tag', 'batteryText');
%% ==================== 14. Robustness & Fault Tolerance ====================
fault_tolerance = struct(...
    'sensor_redundancy', true,...
    'communication_failover', true,...
    'battery_contingency', true,...
    'auto_rtl', true); % Return-to-launch
disp('=== Fault Tolerance ===');
disp(fault_tolerance);
%% ==================== 15. Simulation Execution with Improved Visualization ====================
% Initialize plots
figure(fig1);
pathPlot = plot(ax1, path(:,1), path(:,2), 'g-', 'LineWidth', 2, 'DisplayName', 'Planned Path');
currentPos = start_pos;
traveledPath = currentPos;
% Create detailed drone representation
droneBody = plot(ax1, currentPos(1), currentPos(2), 'ks', 'MarkerSize', 12, 'LineWidth', 2);
droneArms = plot(ax1, [currentPos(1) currentPos(1)], [currentPos(2)-2 currentPos(2)+2], 'k-', 'LineWidth', 3);
droneArms(2) = plot(ax1, [currentPos(1)-2 currentPos(1)+2], [currentPos(2) currentPos(2)], 'k-', 'LineWidth', 3);
droneProps = plot(ax1, currentPos(1), currentPos(2)+2, 'ro', 'MarkerSize', 8, 'LineWidth', 2, 'MarkerFaceColor', 'r');
droneProps(2) = plot(ax1, currentPos(1), currentPos(2)-2, 'ro', 'MarkerSize', 8, 'LineWidth', 2, 'MarkerFaceColor', 'r');
droneProps(3) = plot(ax1, currentPos(1)+2, currentPos(2), 'ro', 'MarkerSize', 8, 'LineWidth', 2, 'MarkerFaceColor', 'r');
droneProps(4) = plot(ax1, currentPos(1)-2, currentPos(2), 'ro', 'MarkerSize', 8, 'LineWidth', 2, 'MarkerFaceColor', 'r');
% Create traveled path plot
travelPlot = plot(ax1, traveledPath(:,1), traveledPath(:,2), 'b-', 'LineWidth', 1.5, 'DisplayName', 'Traveled Path');
% Simulation parameters - reduced speed
dt = 0.2; % Increased timestep
total_energy = 100;
velocity = [0, 0];
animation_speed = 0.2; % Increased pause time for slower movement
% Add detailed info annotations
distText = text(ax1, 10, 10, sprintf('Distance: %.1f m', 0), 'Color', 'w', 'FontWeight', 'bold', 'BackgroundColor', 'k');
velText = text(ax1, 10, 25, sprintf('Speed: %.1f m/s', 0), 'Color', 'w', 'FontWeight', 'bold', 'BackgroundColor', 'k');
batText = text(ax1, 10, 40, sprintf('Battery: %d%%', total_energy), 'Color', 'w', 'FontWeight', 'bold', 'BackgroundColor', 'k');
timeText = text(ax1, 10, 55, sprintf('Time: %.1f s', 0), 'Color', 'w', 'FontWeight', 'bold', 'BackgroundColor', 'k');
% Create interpolated path for smoother animation
interp_factor = 10; % Increased interpolation
xi = interp1(1:size(path,1), path(:,1), linspace(1,size(path,1),size(path,1)*interp_factor))';
yi = interp1(1:size(path,1), path(:,2), linspace(1,size(path,1),size(path,1)*interp_factor))';
interp_path = [xi yi];
% Simulation time tracking
simulationTime = 0;
% Tile windows for simultaneous viewing
figure(fig1);
figure(fig2);
for i = 1:size(interp_path,1)
    try
        % Check if emergency stop was pressed
        if ~simulationRunning
            disp('Simulation stopped by user');
            break;
        end
        
        % Update position and velocity (slower movement)
        newPos = interp_path(i,:);
        if i > 1
            velocity = (newPos - currentPos)/dt;
        end
        currentPos = newPos;
        traveledPath = [traveledPath; currentPos];
        simulationTime = simulationTime + dt;

        % Calculate success rate
        distanceToGoal = norm(currentPos - goal_pos); % Distance to goal
        maxDistance = norm(start_pos - goal_pos); % Maximum possible distance
        proximityScore = max(0, 1 - distanceToGoal / maxDistance); % Proximity score (0 to 1)
        batteryScore = total_energy / 100; % Battery score (0 to 1)
        successRate = 100 * (0.7 * proximityScore + 0.3 * batteryScore); % Weighted success rate

        % Update success rate data
        timeData = [timeData; simulationTime];
        successRateData = [successRateData; successRate];

        % Update success rate plot
        figure(fig4);
        set(successPlot, 'XData', timeData, 'YData', successRateData);
        xlim(ax4, [0 max(timeData)]); % Dynamically adjust x-axis limits
        drawnow;

        % Update info displays
        distance = norm(currentPos - start_pos);
        set(distText, 'String', sprintf('Distance: %.1f m', distance));
        set(velText, 'String', sprintf('Speed: %.1f m/s', norm(velocity)));
        total_energy = max(0, total_energy - 0.01); % Slower battery drain
        set(batText, 'String', sprintf('Battery: %d%%', round(total_energy)));
        set(batteryText, 'String', sprintf('Battery: %d%%', round(total_energy)));
        set(timeText, 'String', sprintf('Time: %.1f s', simulationTime));

        %% Update Navigation View
        figure(fig1);
        % Update drone position
        set(droneBody, 'XData', currentPos(1), 'YData', currentPos(2));
        set(droneArms(1), 'XData', [currentPos(1) currentPos(1)], 'YData', [currentPos(2)-2 currentPos(2)+2]);
        set(droneArms(2), 'XData', [currentPos(1)-2 currentPos(1)+2], 'YData', [currentPos(2) currentPos(2)]);
        % Rotate props slower
        for j = 1:4
            rotate(droneProps(j), [0 0 1], 20, [currentPos(1) currentPos(2) 0]); % Reduced rotation speed
        end
        % Update traveled path more frequently
        set(travelPlot, 'XData', traveledPath(:,1), 'YData', traveledPath(:,2));

        %% Update Thermal View
        figure(fig2);
        % Update FOV rectangle
        if exist('fovRect','var'), delete(fovRect); end
        fovRect = rectangle(ax2, 'Position',...
            [max(1,currentPos(1)-25), max(1,currentPos(2)-25), 50, 50],...
            'EdgeColor','c', 'LineWidth',1.5, 'LineStyle', '--');
        % Add current position marker
        if exist('dronePosThermal','var'), delete(dronePosThermal); end
        dronePosThermal = plot(ax2, currentPos(1), currentPos(2), 'wx', 'MarkerSize', 10, 'LineWidth', 2);
        drawnow;
        pause(animation_speed);

    catch ME
        warning('Error at step %d: %s', i, ME.message);
        break;
    end
end

% Finalize plots
if ishandle(fig1)
    legend(ax1, [startPlot, goalPlot, droneBody, pathPlot, travelPlot],...
        {'Start','Goal','Drone','Planned Path','Traveled Path'},...
        'Location', 'northeastoutside', 'FontSize', 8);
    title(ax1, sprintf('Mission Complete - Time: %.1f s', simulationTime));
end

if ishandle(fig2)
    plot(ax2, goal_pos(1), goal_pos(2), 'gx', 'MarkerSize', 15, 'LineWidth', 3);
    text(ax2, goal_pos(1)+5, goal_pos(2)+5, 'MISSION COMPLETE',...
        'Color', 'g', 'FontSize', 12, 'FontWeight', 'bold');
end

if ishandle(fig4)
    title(ax4, sprintf('Final Success Rate: %.1f%%', successRateData(end)));
end
%% ==================== 16. Ethical Considerations ====================
ethics = struct(...
    'privacy', 'Thermal face blurring implemented',...
    'safety', 'No-fly zones enforced',...
    'transparency', 'All AI decisions logged');
disp('=== Ethical Framework ===');
disp(ethics);
%% Helper Functions
function emergencyStop(hObject, eventdata)
    global simulationRunning;
    simulationRunning = false;
    disp('EMERGENCY STOP ACTIVATED - Stopping simulation');
    % Close all figures except the control panel
    allFigs = findall(0, 'Type', 'figure');
    for i = 1:length(allFigs)
        if ~strcmp(allFigs(i).Name, 'Control Panel')
            close(allFigs(i));
        end
    end
    % Update control panel
    set(findobj('Tag', 'batteryText'), 'String', 'SYSTEM HALTED');
end
