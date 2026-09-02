clear; clc; close all;

%% ---------------- USER PARAMETERS ----------------
% One entry per measurement run. Add/remove rows as needed -- everything
% below loops over this list automatically.
runs = struct( ...
    'file',      {'7.5deg.mat','10deg.mat', '12.5deg.mat'}, ...
    'delta_deg', {7.5,    10,          12.5} );

L           = 0.263;    % [m] wheelbase -- real measured value
g           = 9.81;     % [m/s^2]

% Moving average window durations (in seconds)
w_pos_vel   = [];      % [s] window length for position and velocity smoothing
w_yaw_rate  = [];      % [s] window length for heading and yaw rate smoothing

t_start     = [];      % [s] start time to trim to -- leave [] to keep from t(1)
t_end       = [];      % [s] end time to trim to   -- leave [] to keep to t(end)
dataDir     = [];      % folder containing the .mat files

%% ---------------- Loop over runs ----------------
nRuns = numel(runs);
results = struct('delta_deg', {}, 'vx', {}, 'V2', {}, 'L_eq', {}, 'yaw_gain', {});

for i = 1:nRuns
    fprintf('\n=== Run %d/%d: %s (delta = %g deg) ===\n', i, nRuns, runs(i).file, runs(i).delta_deg);
    %% ---- Load ----
    S = load(fullfile(dataDir, runs(i).file));
    fn = fieldnames(S);
    assert(numel(fn) == 1, ...
        ['Expected exactly one variable in %s, found %d. ' ...
         'Make sure the .mat file only contains the logged SimulationOutput.'], ...
        runs(i).file, numel(fn));
    s = S.(fn{1}); 
    t = s.tout;     % Time [s]
    x = s.x1.data;  % X pos [m]
    y = s.y1.data;  % Y pos [m]
    % Note: x1/y1 are logged as plain arrays (.data), while yaw_deg1 is
    % logged as a timeseries object (.Data) -- the differing capitalization
    % reflects the underlying Simulink logging block, it's not a typo.
    yaw_deg = s.yaw_deg1.Data;
    t = t(:); x = x(:); y = y(:); yaw_deg = yaw_deg(:);

    %% ---- Exercise 3.4: Moving Average Filter + Differentiation ----
    % --- 1) Sampling time -------------------------------------------------
    % TODO: estimate the sampling time dt from the time vector t.
    dt = 0;   % <-- replace

    % --- 2) Window durations [s] -> sample counts -------------------------
    % TODO: convert each window from seconds to an integer number of
    %       samples. Make sure the result is at least 1 sample.
    k_vel = 1;   % <-- replace
    k_yaw = 1;   % <-- replace

    % --- 3) Linear velocity ----------------------------------------------
    % TODO: smooth x and y, differentiate them w.r.t. t to get the global
    %       velocity components, form the speed V, then smooth V.
    x_f  = x;   % <-- replace (smooth)
    y_f  = y;   % <-- replace (smooth)
    vx_c = zeros(size(t));   % <-- replace (d/dt of x_f)
    vy_c = zeros(size(t));   % <-- replace (d/dt of y_f)
    V = zeros(size(t)); % <-- replace (magnitude, then smooth) -- kept only
                         %     as a sanity-check signal for the plot below

    % --- 4) Heading and yaw rate -----------------------------------------
    % TODO: convert yaw_deg to radians and UNWRAP it (see Ex 1.6 / 2.1),
    %       smooth it, differentiate to get omega, then smooth omega.
    psi   = zeros(size(t));   % <-- replace (deg2rad + unwrap + smooth)
    omega = zeros(size(t));   % <-- replace (d/dt of psi, then smooth)

    % --- 5) Body-frame longitudinal velocity -----------------------------
    % TODO: project the global velocity (vx_c, vy_c) onto the heading psi.
    % This vx is the signal actually used from here on (yaw gain, L_eq).
    % Under the kinematic no-slip assumption vx should closely match V
    % above -- comparing the two is a good sanity check on your projection.
    vx = zeros(size(t));   % <-- replace

    %% ---- Trim (after filtering/differentiating) ----
    ts = t_start; te = t_end;
    if isempty(ts), ts = t(1);   end
    if isempty(te), te = t(end); end
    mask = t >= ts & t <= te;

    t = t(mask); vx = vx(mask); V = V(mask); omega = omega(mask); psi = psi(mask);

    %% ---- Effective wheelbase, L_eq(vx) = vx*delta / omega ----
    delta_rad = deg2rad(runs(i).delta_deg);

    % Only keep samples where vx and omega are both large enough to trust:
    %   - vx > 0.1       : at very low speed, differentiated position/heading
    %                       signals are dominated by noise, so vx and omega
    %                       become unreliable
    %   - abs(omega) > eps: L_eq = vx*delta_rad/omega divides by omega, so
    %                       samples with omega ~ 0 would blow up to +-Inf
    %                       or be swamped by noise
    % yaw_gain and L_eq are computed together from this same mask so they
    % stay index-aligned (same samples, same order) for plotting and fitting.
    valid = vx > 0.1 & abs(omega) > eps;

    %% --- Exercise 3.5: Yaw velocity gain ---
    % TODO: Compute the yaw velocity gain. Tip: use omega(valid) to avoid including unreliable low-speed/noisy samples in the fit.
    yaw_gain = [];              % <-- replace

    %% --- Exercise 3.6: Equivalent length ---
    % TODO: Compute L_eq and V^2 using vx(valid), delta_rad, and omega(valid).
    L_eq = [];   % <-- replace
    V2   = [];   % <-- replace

    %% --- Exercise 3.7: Characterization equivalent wheelbase ---
    % TODO: For each steering angle, fit L_eq = a*V2 + b using polyfit on
    % (results(i).V2, results(i).L_eq).
    %
    % Store the fit coefficients, e.g.:
    %   results(i).a_fit = p(1);
    %   results(i).b_fit = p(2);
    % (you'll need to move this into the main loop, or re-loop over `results`)
    %
    % Then, outside the loop:
    %   - print a table comparing a_fit and b_fit across steering angles
    %   - overlay each fitted line on the Ex 3.6 plot for comparison

    %% --- Store results ---
    results(i).delta_deg = runs(i).delta_deg;
    results(i).t         = t;
    results(i).V_full    = V;
    results(i).psi_full  = psi;
    results(i).vx        = vx(valid);
    results(i).V2        = V2;
    results(i).L_eq      = L_eq;
    results(i).yaw_gain  = yaw_gain;
    % If necessary, you can add entries as results(i).[] = ...;

end


%% ---- Exercise 3.4: Plots ----
% V plot
figure('Color','w');
tiledlayout('flow');
for i = 1:nRuns
    nexttile;
    plot(results(i).t, results(i).V_full, 'LineWidth', 1.2);
    grid on;
    xlabel('t  [s]');
    ylabel('V  [m/s]');
    title(sprintf('\\delta = %g^\\circ', results(i).delta_deg));
end
sgtitle('Speed vs time, per run');

% Psi plot
figure('Color','w');
tiledlayout('flow');
for i = 1:nRuns
    nexttile;
    plot(results(i).t, rad2deg(results(i).psi_full), 'LineWidth', 1.2);
    grid on;
    xlabel('t  [s]');
    ylabel('\psi  [deg]');
    title(sprintf('\\delta = %g^\\circ', results(i).delta_deg));
end
sgtitle('Heading vs time, per run');

%% --- Exercise 3.5: Yaw Velocity Gain plot ---
% TODO: On a single figure, plot the measured yaw velocity gain
% (results(i).yaw_gain) against vx for every run, using a different
% marker/color per steering angle.
%
% Then overlay your theoretical expression for yaw_gain(vx) (derived
% using the small-angle approximation tan(delta) ~ delta -- see the
% assignment text) evaluated over the same vx range, using L.

%% --- Exercise 3.6: Combined L_eq vs V^2 plot ---
% TODO: On a single figure, plot results(i).L_eq against results(i).V2
% for every run (one series per steering angle).
%
% Add a horizontal reference line at y = L (the measured wheelbase) using
% yline(L, '--', 'measured L') so you can visually compare L_eq against
% the true wheelbase.

%% --- Exercise 3.7: Characterization equivalent wheelbase plot ---
% TODO: print a table comparing a_fit and b_fit across steering angles