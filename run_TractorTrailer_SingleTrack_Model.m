close all
clc
clearvars

%% Truck&Trailer geometry
params.geometry.L1 = .263;             % Wheelbase of the tractor [m]
params.geometry.L2 = .572;             % Wheelbase of the trailer [m]
params.geometry.L1c = .051;            % distance from the rear axle to hitch/king pin position [m]

%% Input parametrs
params.input.steering_angle = 20;      % Steering angle of tractor front wheel [deg]
params.input.velocity = 0.05;          % Longitudinal velocity of the tractor [m/s]
% Initial conditions (PLEASE USE THEM IN INTEGRATOR BLOCKS)
params.init.x10 = 0;                    % inital condition for x1 pose of the tractor rear axle [m]
params.init.y10 = 0;                    % inital condition for y1 pose of the tractor rear axle [m]
params.init.psi10 = deg2rad(0);         % initial orientation/heading of the tractor [rad]
params.init.psi20 = deg2rad(0);         % initial orientation/heading of the trailer [rad]

%% Tractor Kinematic Center-Of-Mass (COM) Single track model with side slip angle beta
tractor_model = "Tractor_Trailer_SingleTrack_Simulink";
sim_obj = Simulink.SimulationInput(tractor_model);

sim_obj = setModelParameter(sim_obj,...
            'StopTime', '200', ...
            'Solver', 'ode45',...
            'FixedStep', '.1');
        
results = sim(sim_obj);
%% Tractor COM location simulation

% Extract timeseries data from simulation output
x_data = results.x1_pose.Data;
y_data = results.y1_pose.Data;
yaw1_data = results.yaw1.Data;
yaw2_data = results.yaw2.Data;
art_data = results.articulation.Data;
t_data = results.sim_time.Time;

fig1 = figure(1);
theme(fig1, 'light')
hold on;
grid on;
axis equal;
xlabel('X Position [m]');
ylabel('Y Position [m]');
title('Tractor Simulation');

trajectory_plot = plot(NaN, NaN, 'b-', 'LineWidth', 1.5);    % Trajectory line
rear_axle_traj = plot(NaN, NaN, 'k--', 'LineWidth', 2);
rear_axle_trailer_traj = plot(NaN, NaN, 'b--', 'LineWidth', 2);
current_hitch_pose = plot(NaN, NaN, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r'); % Current position of COM marker
vehicle_heading_plot = plot(NaN, NaN, 'r-', 'LineWidth', 2); % Current vehicle heading (orientation)
trailer_orientation_plot = plot(NaN, NaN, 'g-', 'LineWidth', 2);
rear_axle_trailer_check = plot(NaN, NaN,'MarkerSize', 15, 'MarkerFaceColor', 'm', 'Marker', 'pentagram');


x2_rear = [];
y2_rear = [];

% Animation loop
for k = 1:length(t_data)
    % Update trajectory so far
    set(trajectory_plot, 'XData', x_data(1:k), 'YData', y_data(1:k));
    % Current position of the rear axle and orientation
    x1 = x_data(k);
    y1 = y_data(k);
    psi_1 = yaw1_data(k);
    psi_2 = yaw2_data(k);
    delta = art_data(k);
    
    % Current hitch/king in position ALSO FRONT OF THE TRAILER
    x_hitch = x1 + params.geometry.L1c*cos(psi_1);
    y_hitch = y1 + params.geometry.L1c*sin(psi_1);
    
    % Update current COM pose with dot marker 
    set(current_hitch_pose, 'XData', x_hitch, 'YData', y_hitch);
    
    % Update current pose and orientation of the tractor with a line based on yaw data
    x_front = x1 + params.geometry.L1*cos(psi_1);
    y_front = y1 + params.geometry.L1*sin(psi_1);

    set(rear_axle_traj, 'XData', x_data(1:k), 'YData', y_data(1:k));
    
    % Trailer rear axle position
    x2_r = x_hitch - params.geometry.L2*cos(psi_2);
    y2_r = y_hitch - params.geometry.L2*sin(psi_2);
    
    % Trailer rear check FIGURE OUT WHY THE MINUS DELTA
    x2_r_ch = x1 + params.geometry.L1c*cos(psi_1) - params.geometry.L2*(cos(-delta)*cos(psi_1) - sin(-delta)*sin(psi_1));
    y2_r_ch = y1 + params.geometry.L1c*sin(psi_1) - params.geometry.L2*(cos(-delta)*sin(psi_1) + sin(-delta)*cos(psi_1));
    
    set(rear_axle_trailer_check, 'XData' , x2_r_ch, 'YData', y2_r_ch);
    
    x2_rear(end+1) = x2_r;
    y2_rear(end+1) = y2_r;
    
    set(rear_axle_trailer_traj, 'XData', x2_rear, 'YData', y2_rear);
    
    % Update vehicle symetry plane
    set(vehicle_heading_plot, 'XData', [x1, x_front], 'YData', [y1, y_front]);
    set(trailer_orientation_plot, 'XData', [x2_r, x_hitch], 'YData', [y2_r, y_hitch]);
    
    % Pause to simulate real time (optional: adjust speed)
    pause(0.05);
    
end


%% SANITY CHECKS 
% fit a circle over the rear axle data pose TRACTOR
[a1,b1,r1] = helpers.fit_circle(x_data(10:end), y_data(10:end));
% fit a circle over the rear axle data pose TRACTOR
[a2,b2,r2] = helpers.fit_circle(x2_rear(15:end).', y2_rear(15:end).');

% Est. distance between the origin of the fitted circle and end pose of rear axle
dist1 = helpers.compute_radius(a1, b1, x_data(end), y_data(end));
dist2 = helpers.compute_radius(a2, b2, x2_rear(end), y2_rear(end));
% check if radius perpendicular to the end orientation of the vehicle
[m_tractor, m_tractor_radius, parpendicular_tractor] = helpers.compute_slope("TRACTOR" , a1, b1,  x_data(end), y_data(end), yaw1_data(end));
[m_trailer, m_trailer_radius, parpendicular_trailer] = helpers.compute_slope("TRAILER" , a2, b2,  x2_rear(end), y2_rear(end), yaw2_data(end));

helpers.check_perpendicularity("TRACTOR", a1, b1, x_data(end), y_data(end), yaw1_data(end));
helpers.check_perpendicularity("TRAILER", a2, b2, x2_rear(end), y2_rear(end), yaw2_data(end));

%% Figure 2: Rear axle trajectory + fitted circle
fig2 = figure(2);
theme(fig2, 'light')
hold on;
grid on;
axis equal;

xlabel('X Position [m]');
ylabel('Y Position [m]');
title('Rear Axle Trajectory and Fitted Circle');

% Plot trajectory
plot(x_data, y_data, 'k--', 'LineWidth', 1.5, 'DisplayName', 'Rear Axle Trajectory');

% Scatter points
scatter(x_data, y_data, 25, 'filled', 'MarkerFaceColor', 'b', 'DisplayName', 'Data Points');
scatter(x2_rear, y2_rear, 25, 'filled', 'MarkerFaceColor', 'c', 'DisplayName', 'Data Points');

% Plot fitted circle 
theta = linspace(0, 2*pi, 200);
x_circle = a1 + r1*cos(theta);
y_circle = b1 + r1*sin(theta);
plot(x_circle, y_circle, 'r-', 'LineWidth', 2, 'DisplayName', 'Fitted Circle: TRACTOR');

x_circle_trailer = a2 + r2*cos(theta);
y_circle_trailer = b2 + r2*sin(theta);
plot(x_circle_trailer, y_circle_trailer, 'm-', 'LineWidth', 1, 'DisplayName', 'Fitted Circle: TRAILER');

% Plot circle center
plot(a1, b1, 'r+', 'MarkerSize', 15, 'LineWidth', 2, 'DisplayName', 'Circle Center: TRACTOR');
plot(a2, b2, 'm+', 'MarkerSize', 15, 'LineWidth', 2, 'DisplayName', 'Circle Center: TRAILER');
plot(x_data(end), y_data(end), 'g.', 'MarkerSize', 20, 'DisplayName', 'Last Pose: TRACTOR');
plot([x_data(end) x_data(end)+ .263*cos(yaw1_data(end))], [y_data(end) y_data(end)+ .263*sin(yaw1_data(end))], 'g-', 'LineWidth', 3,'DisplayName', 'TRACTOR');
plot([x_data(end) a1], [y_data(end) b1], 'g--', 'LineWidth', 2, 'DisplayName', "TRACTOR radius");

plot(x2_rear(end), y2_rear(end), 'k.', 'MarkerSize', 20, 'DisplayName', 'Last Pose: TRACTOR');
plot([x2_rear(end) x2_rear(end)+ .572*cos(yaw2_data(end))], [y2_rear(end) y2_rear(end)+ .572*sin(yaw2_data(end))], 'k-', 'LineWidth', 3,'DisplayName', 'TRACTOR');
plot([x2_rear(end) a2], [y2_rear(end) b2], 'k--', 'LineWidth', 2, 'DisplayName', "TRACTOR radius");

legend show;

%% Plotting
% Exercise 2.2: Quantitative kinematic model analysis
%%%%%%%%%%%% YOUR CODE: START %%%%%%%%%%%%%

%%%%%%%%%%%%  YOUR CODE: END  %%%%%%%%%%%%%
%% stationary rear axle trailer
steering_angle = atan(params.geometry.L1/ params.geometry.L2);
steering_angle_deg = rad2deg(steering_angle);
disp(steering_angle_deg); 



