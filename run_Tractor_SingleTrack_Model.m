close all
clc
clearvars

%% Input parametrs
params.input.steering_angle = 15;   % Steering angle of tractor front wheel [deg]
params.input.velocity = 0.05;          % Longitudinal velocity of the tractor [m/s]

%% Tractor Kinematic Center-Of-Mass (COM) Single track model with side slip angle beta
tractor_model = "Tractor_SingleTrack_Simulink";
sim_obj = Simulink.SimulationInput(tractor_model);

sim_obj = setModelParameter(sim_obj,...
            'StopTime', '100', ...
            'Solver', 'ode45',...
            'FixedStep', '.1');
        
results_1 = sim(sim_obj);
%% Tractor animation

% Extract timeseries data from simulation output
x_data = results_1.x1_pose.Data;
y_data = results_1.y1_pose.Data;
yaw_data = results_1.yaw1.Data;
t_data = results_1.sim_time.Time;

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
current_hitch_pose = plot(NaN, NaN, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r'); % Current position of COM marker
vehicle_heading_plot = plot(NaN, NaN, 'r-', 'LineWidth', 2); % Current vehicle heading (orientation)


% Animation loop
for k = 1:length(t_data)
    % Update trajectory so far
    set(trajectory_plot, 'XData', x_data(1:k), 'YData', y_data(1:k));
    % Current position of the rear axle and orientation
    x = x_data(k);
    y = y_data(k);
    psi = yaw_data(k);

    % Current hitch/king in position
    x_hitch = x + TruckParams.L1c*cos(psi);
    y_hitch = y + TruckParams.L1c*sin(psi);

    % Update current COM pose with dot marker 
    set(current_hitch_pose, 'XData', x_hitch, 'YData', y_hitch);

    % Update current pose and orientation of the tractor with a line based on yaw data
    x_front = x + TruckParams.L1*cos(psi);
    y_front = y + TruckParams.L1*sin(psi);

    set(rear_axle_traj, 'XData', x_data(1:k), 'YData', y_data(1:k));

    % Update vehicle symetry plane
    set(vehicle_heading_plot, 'XData', [x, x_front], 'YData', [y, y_front]);
    pause(0.05);
end

%% Computational analysis
% fit a circle over the rear axle data pose TRACTOR
[a1,b1,r1] = helpers.fit_circle(x_data(10:end), y_data(10:end));

% Est. distance between the origin of the fitted circle and end pose of
% rear axle; THEN compare the radius 'r' of the fitted circle
dist = helpers.compute_radius(a1, b1, x_data(end), y_data(end));
% Verify if the turning radius at the rear axle and vehicle heading are perpendicular
[m_tractor, m_tractor_radius, parpendicular_tractor] = helpers.compute_slope("TRACTOR" , a1, b1,  x_data(end), y_data(end), yaw_data(end));
% Check perpendicularity of the vehicle's heading and its turing radius
helpers.check_perpendicularity("TRACTOR", a1, b1, x_data(end), y_data(end), yaw_data(end));

%% Figure 2: Rear axle trajectory + fitted circle
fig2 = figure(2);
theme(fig2, 'light')
hold on; grid on; axis equal;
xlabel('X Position [m]'); ylabel('Y Position [m]'); title('Rear Axle Trajectory and Fitted Circle');
% plot trajectory
plot(x_data, y_data, 'k--', 'LineWidth', 1.5, 'DisplayName', 'Rear Axle Trajectory');
% scatter points
scatter(x_data, y_data, 25, 'filled', 'MarkerFaceColor', 'b', 'DisplayName', 'Data Points');
% plot fitted circle
helpers.draw_circle(a1, b1, r1, 'r-')
% Plot circle center
plot(a1, b1, 'r+', 'MarkerSize', 15, 'LineWidth', 2, 'DisplayName', 'Circle Center');
plot(x_data(end), y_data(end), 'g.', 'MarkerSize', 20);
plot([x_data(end) x_data(end)+ TruckParams.L1*cos(yaw_data(end))], [y_data(end) y_data(end)+ TruckParams.L1*sin(yaw_data(end))], 'g-', 'LineWidth', 3);
plot([x_data(end) a1], [y_data(end) b1], 'g--', 'LineWidth', 2);

legend show; hold off;


%% Plotting

% Exercise 1.6: Tractor heading angle analysis
% Plot the tractor heading angle psi1 over time
%%%%%%%%%%%% YOUR CODE: START %%%%%%%%%%%%%
fig3 = figure
theme(fig3, "light")
plot(t_data, yaw_data)
xlabel("Time [s]"); ylabel("heading angle [rad]"); title("Tractor Heading Angle");
%%%%%%%%%%%%  YOUR CODE: END  %%%%%%%%%%%%%

