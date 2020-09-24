function default_options = defaultOptions()
% This function defines default options for the safe learning demo.

% Choose reachability and learning features
default_options.dry_run = 1; % When 1, runs code without communications
default_options.online_reach = 0; % When to compute new reach set
default_options.use_trajopt = 0; % Plan open-loop trajectory using trajopt
default_options.use_pgsd = 0; % Use either PGSD or LQR control

% Visualize computations
default_options.plot_traj = 0; % Plot trajectory for dry run
default_options.plot_disturb = 1; % Plot measured and filtered disturbance

% Choose type of disturbance for dry run
% Options: zeros, ones, linear, fan
default_options.real_d = 'fan'; 

% Define options for LQR
default_options.Q = diag([10 5 0].^2);
default_options.R = 1;
default_options.track_open_loop = 1; % Build controller around open loop model-based control

% Safety transistion region
default_options.pred_time = 0.2; % Reachability lookahead to determine control action
default_options.eta_u = 0.1; % Fraction of relative safety at which transition to u* begins
default_options.eta_l = 0.0; % Fraction of relative safety at which transition to u* ends

% Choose type of reference trajectory
% Options: training, testing, descent, sin, smooth, steps, training_new
default_options.ref_mode = 'training_new';

% Generating a new model
default_options.test_run_duration = 100; % Length of test run in seconds
default_options.points_for_new_model = 600; % Data points needed to create disturbance model

% Number of points to smooth (average) the disturbance signal over
default_options.n_disturb_pts = 30;

% Minimum points needed to contract safe set
default_options.pts_update_reach = 10;

% Threshold for contracting safe set
default_options.lambda_threshold = 0.1;

end
