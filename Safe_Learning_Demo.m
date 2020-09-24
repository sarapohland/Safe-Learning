%% Safe_Learning_Demo
% Run this file to demonstrate the functionality of the safe learning
% framework.

%% Define model and default parameters
% Generate initial model parameters
model = modelParameters();

%% Generate function of min and max disturbance vs altitude
% d_max = [lowest safe altitude    max disturbance at altitude]
%         [        ...                     ...                ]
%         [highest safe altitude   max disturbance at altitude]
model.length_disturb_model = floor(2 + (model.target(2, 1) - model.target(1, 1)) * 100);
model.d_max = [(linspace(model.target(1, 1), model.target(2, 1), model.length_disturb_model)).', ...
    ones(model.length_disturb_model, 1) * model.default_d_max];
model.d_min = [(linspace(model.target(1, 1), model.target(2, 1), model.length_disturb_model)).', ...
    ones(model.length_disturb_model, 1) * model.default_d_min];

% Define default operation
options = defaultOptions();

%% Complete precomputation
% Compute initial reachability analysis
[reachability.valueFunc, reachability.grid] = calcReachSet(model);
reachability.gradient = computeGradients(reachability.grid, reachability.valueFunc);
reachability.obj = QuadVerticalFlight(model, 1:2);

% Create model cell array
if isstruct(model)
    model_cell{1} = model;
else
    model_cell = model;
end

% Compute LQR gains based on initial model
try
    options.K_lqr{1} = quadLQR(model_cell{1}, options);
catch
    options.K_lqr{1} = [150 50];
end

% Compute starting altitude and velocity
start_pt = getRefAlt(0, options);

% Generate "real" disturbance for dry run
if options.dry_run
    model.real_d = realDisturb(model, options);
end

%% Prepare to start test
% Pre-test hover flight
if ~options.dry_run
    % send commnad to hover at start_pt
end

% Ask for user prompt to begin test
if ~options.dry_run
    fprintf('\nReady to run test. \n\nGet quadrotor hovering at initial altitude and press any key...')
    pause();
end

% Initialize traces
initial_time = now;
traces.state = [0, model.x1_init, model.x2_init]; % [time, altitude(x1), velocity(x2)]
traces.accel = []; % [time, observed acceleration]
traces.ctrl = []; % [time, control]
traces.disturb = []; % [time, measured disturbance]
traces.filtered_disturb = []; % [time, filtered disturbance]
traces.ref = []; % [reference trace]
traces.int_error = []; % [time, integral error]
traces.reachValZero = []; % [level set]

%% Run test to collect data
% Perform test until you have enough data
data_pts_collected = size(traces.state, 1);
iteration = 0;
while data_pts_collected < options.points_for_new_model
    iteration = iteration + 1;
    traces = collectData(initial_time, traces, model, options, reachability);
    data_pts_collected = size(traces.state, 1);
    fprintf('Completed %d iteration(s) of test!\n', iteration);
end

% Optional: Plot state to visualize dry run
if options.plot_traj
    plotTraj(traces);
end

% Optional: Plot disturbance vs altitude
if options.plot_disturb
    plotDisturb(traces);
end

%% Generate model of disturbance and update reachable set

gaussian = modelDisturb(traces, model);
[model, reachability] = updateReachSet(model, gaussian, ...
    reachability.valueFunc);

