clearvars;
close all;
clc;

%% Parameters
% Truck&Trailer geometry
params.geometry.L1 = .263;              % Wheelbase of the tractor [m]
params.geometry.L2 = .572;              % Wheelbase of the trailer [m]
params.geometry.L1c = .051;             % distance from the rear axle to hitch/king pin position [m]

% Initial conditions
params.init.x10 = 0;                    % inital condition for x1 pose of the tractor rear axle [m]
params.init.y10 = 0;                    % inital condition for y1 pose of the tractor rear axle [m]
params.init.psi10 = deg2rad(0);         % initial orientation/heading of the tractor [rad]
params.init.psi20 = deg2rad(-90);         % initial orientation/heading of the trailer [rad]

% Input parameters
params.input.steering_angle = 15;       % REQUESTED steering angle (from Pure Pursuit / Ackermann) [deg]
params.input.velocity = 0.15;            % Longitudinal velocity of the tractor [m/s]

% Simulation
tmax_kin = 500;          % Kinematic model simulation time [s]
tmax_md  = 60;           % Manual Driving simulation time [s]
fs  = 120;               % sample frequency for data storage [Hz] =>controller rate 120 Hz
Ts  = 1./fs;

% --- Data trimming ranges (sample indices) --------------------------------
% Use these to exclude the startup transient (spiral-in before the vehicle
% settles onto the steady circular path) before fitting circles. The
% trailer needs a later start index than the tractor since it lags behind
% and takes longer to converge. Set "_end" to Inf to keep to the end.
trim.kin.tractor_start = 10;    trim.kin.tractor_end = Inf;
trim.kin.trailer_start = 15;    trim.kin.trailer_end = Inf;

% --- MD data trimming range (time-based, seconds) --------------------------
% Keeps t_start <= t <= t_track(end) - end_offset, i.e. trims off the
% startup transient at t_start and the last "end_offset" seconds of the
% log (e.g. deceleration/stop at the end of the run).
trim.md.tractor_tstart = 5;     trim.md.end_offset = 5;   % <- tune after inspecting the scatter
trim.md.trailer_tstart = 5;

%% Load OptiTrack data
% NOTE: measurements are now saved as a single Simulink.SimulationOutput
% object per file (not the old "md" struct). Each object logs the
% timeseries:
%   x1,y1,psi1  -> tractor rear-axle pose  (psi in rad)
%   x2,y2,psi2  -> trailer rear-axle pose  (psi in rad)
%   tout        -> simulation/measurement time vector [s]
% plus the raw OptiTrack rigid bodies:
%   xpos,ypos,zpos,yaw_deg     -> body 1 (tractor)
%   xpos1,ypos1,zpos1,yaw_deg1 -> body 2 (trailer)
meas_file = "15deg15msexample.mat";   % <-- set this to the OptiTrack log you saved in Exercise 3.2 (must be on the MATLAB path or in this folder)
opt_raw = load(meas_file);

% Grab the SimulationOutput object regardless of the variable name it was saved under
opt_fields = fieldnames(opt_raw);
md = opt_raw.(opt_fields{1});   % Simulink.SimulationOutput of the manual drive

% --- Guard 1: reject runs that ended with a Simulink error ---------------
% A failed/aborted export stores a non-empty ErrorMessage (and no usable data).
if isa(md,'Simulink.SimulationOutput') && ~isempty(md.ErrorMessage)
    error('meas:erroredRun', ...
        ['The run in "%s" ended with a Simulink error and holds no usable ' ...
         'trajectory:\n  %s\nUse a clean run (empty ErrorMessage) instead.'], ...
         meas_file, md.ErrorMessage);
end

%%
% Single track Tractor-Trailer kinematic model
tractor_trailer_model = "Tractor_Trailer_SingleTrack_Simulink";
sim_obj1 = Simulink.SimulationInput(tractor_trailer_model);

sim_obj1 = setModelParameter(sim_obj1,...
            'StopTime', num2str(tmax_kin), ...
            'Solver', 'ode45');...
            % 'FixedStep', num2str(Ts));

kin = sim(sim_obj1);

%%
% MANUAL DRIVE: Extracting the relevant simulation data (new format)
x1_opti = md.x1.Data(:); y1_opti = md.y1.Data(:); psi1_opti = md.psi1.Data(:);
x2_opti = md.x2.Data(:); y2_opti = md.y2.Data(:); psi2_opti = md.psi2.Data(:);
t_md    = md.tout(:);
t_track = md.x1.Time(:);

% --- Guard 2: reject empty / degenerate trajectories ----------------------
finite_mask = isfinite(x1_opti) & isfinite(y1_opti);
if numel(x1_opti) < 10 || nnz(finite_mask) < 10
    error('meas:noTrajectory', ...
        ['Measurement "%s" has no usable tractor trajectory (%d finite ' ...
         'samples). This is typically an errored or empty export.'], ...
         meas_file, nnz(finite_mask));
end

% Measured longitudinal speed of the tractor from the OptiTrack trajectory,
% used only as an independent sanity check on the nominal speed above.
% It is NOT fed into the model, so a noisy/stationary log can never inject
% a NaN into the solver.
dt      = diff(t_track);
v_inst  = hypot(diff(x1_opti), diff(y1_opti)) ./ dt;
v_meas  = median(v_inst(isfinite(v_inst) & v_inst > 0));   % robust to OptiTrack noise/gaps
if isfinite(v_meas) && v_meas > 0
    fprintf('Speed check for %s : nominal %.4f m/s | measured %.4f m/s\n', ...
            meas_file, params.input.velocity, v_meas);
else
    warning('meas:noSpeed', ...
        ['Could not estimate a valid speed from "%s" (got %g) - the tractor ' ...
         'barely moved in this log. Proceeding with the nominal %.4f m/s; ' ...
         'the circle fit for this run is unlikely to be meaningful.'], ...
         meas_file, v_meas, params.input.velocity);
end
% To instead drive the model at the measured speed for a clean run, uncomment:
% if isfinite(v_meas) && v_meas > 0, params.input.velocity = v_meas; end

% KIN MODEL: Extract timeseries data from simulation output
x1_kin = kin.x1_pose.Data; y1_kin = kin.y1_pose.Data; psi1_kin = kin.yaw1.Data; t_kin = kin.sim_time.Time;
psi2_kin = kin.yaw2.Data;
% Hitch (kingpin) position (vectorized)
x_hitch = x1_kin + params.geometry.L1c*cos(psi1_kin);
y_hitch = y1_kin + params.geometry.L1c*sin(psi1_kin);

% Trailer rear axle (vectorized)
x2_kin = x_hitch - params.geometry.L2*cos(psi2_kin);
y2_kin = y_hitch - params.geometry.L2*sin(psi2_kin);

% Trim the simulation data to only include the circular path steady state
kin_tractor_idx = trim_range(trim.kin.tractor_start, trim.kin.tractor_end, numel(x1_kin));
kin_trailer_idx = trim_range(trim.kin.trailer_start, trim.kin.trailer_end, numel(x2_kin));
x1_kin_trim = x1_kin(kin_tractor_idx); y1_kin_trim = y1_kin(kin_tractor_idx);
x2_kin_trim = x2_kin(kin_trailer_idx); y2_kin_trim = y2_kin(kin_trailer_idx);

% Trim the manual-drive data by time: [t_start, t_track(end) - end_offset]
% (tune trim.md.* above based on the scatter)
t_end_md = t_track(end) - trim.md.end_offset;
md_tractor_idx = trim_range_time(t_track, trim.md.tractor_tstart, t_end_md);
md_trailer_idx = trim_range_time(t_track, trim.md.trailer_tstart, t_end_md);
x1_opti_trim = x1_opti(md_tractor_idx); y1_opti_trim = y1_opti(md_tractor_idx);
x2_opti_trim = x2_opti(md_trailer_idx); y2_opti_trim = y2_opti(md_trailer_idx);

% KIN MODEL:
% fit a circle over the rear axle data pose TRACTOR
[a1_kin, b1_kin, r1_kin] = fit_circle(x1_kin_trim, y1_kin_trim);
% fit a circle over the rear axle data pose TRAILER
[a2_kin, b2_kin, r2_kin] = fit_circle(x2_kin_trim, y2_kin_trim);

% MANUAL DRIVE:
% fit a circle over the rear axle data pose TRACTOR
[a1_md, b1_md, r1_md] = fit_circle(x1_opti_trim, y1_opti_trim);
% fit a circle over the rear axle data pose TRAILER
[a2_md, b2_md, r2_md] = fit_circle(x2_opti_trim, y2_opti_trim);

% ZERO-MEAN all the data
x1k00 = x1_kin_trim - a1_kin; y1k00 = y1_kin_trim - b1_kin;
x2k00 = x2_kin_trim - a1_kin; y2k00 = y2_kin_trim - b1_kin; % <- subtracting (a1,b1) becasue we want to keep any relative diffrentces between the fitted TRACTOR-TRAILER circles

x1m00 = x1_opti_trim - a1_md; y1m00 = y1_opti_trim - b1_md;
x2m00 = x2_opti_trim - a1_md; y2m00 = y2_opti_trim - b1_md;

% fit circle again now on the zeroed out data
% KIN MODEL:
% fit a circle over the rear axle data pose TRACTOR
[a1_kin00, b1_kin00, r1_kin00] = fit_circle(x1k00, y1k00);
% fit a circle over the rear axle data pose TRAILER
[a2_kin00, b2_kin00, r2_kin00] = fit_circle(x2k00, y2k00);

% MANUAL DRIVE:
% fit a circle over the rear axle data pose TRACTOR
[a1_md00, b1_md00, r1_md00] = fit_circle(x1m00, y1m00);
% fit a circle over the rear axle data pose TRAILER
[a2_md00, b2_md00, r2_md00] = fit_circle(x2m00, y2m00);

%% Plotting

% Plot the fitted circles for both kinematic and manual drive data
fig1 = figure(1);
theme(fig1, 'light')
hold on; grid on; axis equal;
title('Fitted Circles (Zero-out at Tractor Center)'); xlabel('X [m]'); ylabel('Y [m]');
% Zero-mean scatters
scatter(x1k00, y1k00, 15, 'b', 'filled', 'DisplayName','KIN: tractor (zero-mean)');
scatter(x2k00, y2k00, 25, 'c', 'filled', 'DisplayName','KIN: trailer (zero-mean)');
scatter(x1m00, y1m00, 15, 'r', 'filled', 'DisplayName','MD: tractor (zero-mean)');
scatter(x2m00, y2m00, 15, 'm', 'filled', 'DisplayName','MD: trailer (zero-mean)');
% Fitted circles (zero-mean)
plot_circle(a1_kin00, b1_kin00, r1_kin00, 'g', 'KIN Tractor (zero-mean)');
plot_circle(a2_kin00, b2_kin00, r2_kin00, 'r', 'KIN Trailer (zero-mean)');
plot_circle(a1_md00,  b1_md00,  r1_md00, 'b', 'MD Tractor (zero-mean)');
plot_circle(a2_md00,  b2_md00,  r2_md00, 'k', 'MD Trailer (zero-mean)');
legend('Location','best');
hold off;

fig2 = figure(2);
theme(fig2, 'light')
hold on; grid on; axis equal;
title('Fitted Circles - Tractor Only (Zero-out at Tractor Center)'); xlabel('X [m]'); ylabel('Y [m]');
scatter(x1k00, y1k00, 15, 'b', 'filled', 'DisplayName','KIN: tractor (zero-mean)');
scatter(x1m00, y1m00, 15, 'r', 'filled', 'DisplayName','MD: tractor (zero-mean)');
plot_circle(a1_kin00, b1_kin00, r1_kin00, 'g', 'KIN Tractor (zero-mean)');
plot_circle(a1_md00,  b1_md00,  r1_md00, 'b', 'MD Tractor (zero-mean)');
legend('Location','best');
hold off;

%% quick turning radius compute
R = params.geometry.L1/tan(deg2rad(params.input.steering_angle));

%% === Sub-functions ===
function idx = trim_range(start_idx, end_idx, n)
%TRIM_RANGE Build a valid index vector [start_idx:end_idx] clipped to data length n.
% end_idx may be Inf (or empty) to mean "to the end of the data".
if isempty(end_idx) || isinf(end_idx)
    end_idx = n;
end
start_idx = max(1, round(start_idx));
end_idx   = min(n, round(end_idx));
if start_idx > end_idx
    error('trim_range:invalidRange', ...
        'Trim start (%d) is after trim end (%d) for data of length %d.', ...
        start_idx, end_idx, n);
end
idx = start_idx:end_idx;
end

function idx = trim_range_time(t, t_start, t_end)
%TRIM_RANGE_TIME Build an index vector selecting samples with t_start <= t <= t_end.
t = t(:);
idx = find(t >= t_start & t <= t_end);
if isempty(idx)
    error('trim_range_time:invalidRange', ...
        'No samples fall within [%.4f, %.4f] s (data spans [%.4f, %.4f] s).', ...
        t_start, t_end, t(1), t(end));
end
end

function [a,b,r] = fit_circle(x_data,y_data)
%FIT_CIRCLE Least-squares circle fit over rear-axle (x,y) pose data.
% Ensure column vectors so the design matrix is built correctly
x_data = x_data(:);
y_data = y_data(:);
% fit a circle over the rear axle data pose
A = [x_data, y_data, ones(size(x_data))];
b = -x_data.^2 - y_data.^2;

% Solve Least Squares problem A * [D, E, f].T = b;
params = A\b;

D = params(1);
E = params(2);
F = params(3);

% Recover a b coeffs and radius
a = -D/2;
b = -E/2;
r = sqrt(a^2 + b^2 - F);
end

function plot_circle(a, b, r, style, name)
    % Plot fitted circle
    theta = linspace(0, 2*pi, 256);
    x_circle = a + r*cos(theta);
    y_circle = b + r*sin(theta);
    disp_name_str = sprintf('Fitted Circle: %s', name);
    plot(x_circle, y_circle, style, 'LineWidth', 2, 'DisplayName', disp_name_str);
end
