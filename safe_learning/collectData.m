function traces = collectData(initial_time, traces, model, options, reachability)
% Function used to collect data during safe learning test. Receives
% information about state of system to calculate disturbance and implement
% control that combines a safety controller and performance controller.

fprintf('Running test...\n');

%% Set up running time
num_sec_in_day = 24 * 60 * 60;
test_duration = options.test_run_duration / num_sec_in_day;
start_time = now;
current_time = now;

%% Run test for a given time duration
while current_time < start_time + test_duration
    %% Time at which you're recording data
    data_time = (current_time - initial_time) * num_sec_in_day;
    ref_time = (current_time - start_time) * num_sec_in_day;

    %% Check if current state is safe
    reachVal = eval_u(reachability.grid, reachability.valueFunc, ...
        [traces.state(end, 2) traces.state(end, 3)]);
    if reachVal > 0
        fprintf('\n');
        warning('Current state is unsafe.');
        fprintf('\n');
    end
    
    %% Calculate acceleration and disturbance
    if size(traces.state, 1) >= 3
        % Approximate derivative of velocity (x2) with respect to time 
        accel_observed = (traces.state(end, 3) - traces.state(end-1, 3)) / ...
            (traces.state(end,1) - traces.state(end-1,1));
        % Expected derivative of veolcity (x2) without disturbance
        accel_expected = (model.k_T * traces.ctrl(end, 2)) + model.g + model.k_0;
        % Disturbance is added to the vertical acceleration
        disturbance = accel_observed - accel_expected;
        
        % Record observed acceleration (unless time has not changed)
        if size(traces.accel, 1) < 1 || data_time > traces.accel(end, 1)
            current_accel = [data_time, accel_observed];
            traces.accel = [traces.accel; current_accel];
        end
        
        % Record disturbance (unless time has not changed)
        if size(traces.disturb, 1) < 1 || data_time > traces.disturb(end, 1)
            current_disturb = [data_time, disturbance];
            traces.disturb = [traces.disturb; current_disturb];
        end
    end

    %% Filter Disturbance
    if size(traces.disturb, 1) >= 1
        filtered_disturbance = filter(1 / options.n_disturb_pts * ...
            ones(1, options.n_disturb_pts), 1, (traces.disturb(:, 2)));
        
        filtered_disturbance = filloutliers(filtered_disturbance, 'previous');
        
        % Record disturbance (unless time has not changed)
        if size(traces.filtered_disturb, 1) < 1 || data_time > traces.filtered_disturb(end, 1)
            traces.filtered_disturb = filtered_disturbance;
        end
    end
    
    %% Contract safe set if necessary
    if size(traces.disturb, 1) >= options.pts_update_reach
        % Find observed and expected disturbance
        d_obs = filtered_disturbance(end); % observed disturbance
        d_index = find(int32(100 * model.d_max(:, 1)) == int32(100 * traces.state(end, 2)));
        if d_index <= length(model.d_max(:, 2))
            d_max = model.d_max(d_index(1), 2); % max disturbance expected
            d_min = model.d_min(d_index(1), 2); % min disturbance expected
        end
         
        % Calculate reachable set value
        reachVal = interpn(reachability.grid.vs{1}, reachability.grid.vs{2}, ...
            reachability.valueFunc, traces.state(end, 2), traces.state(end, 3));
               
        % Compare observed and expected disturbance
        lambda = min(d_obs - d_min, d_max - d_obs) / max(0.1, ((d_max - d_min) / 2));

        if lambda <= options.lambda_threshold
            traces.reachValZero(end+1) = min(traces.reachValZero(end), reachVal);
            if size(traces.reachValZero, 1) < 1 || traces.reachValZero(end) ~= traces.reachValZero(end-1)
                fprintf('Contracting safe set to level set V = %.2f\n', traces.reachValZero(end));
            end
        else
            traces.reachValZero(end+1) = traces.reachValZero(end);
        end
        
    else
        traces.reachValZero(end+1) = 0;
    end
    
    
    %% Get performance controller (using LQR control)
    % Get reference altitude and open loop control
    traces.ref(end+1, :) = getRefAlt(ref_time, options);
    open_loop_ctrl = model.u_hov;
    
    % Use LQR controller if not using PGSD
    if ~options.use_pgsd
        v_lqr = -options.K_lqr{end} * (traces.state(end, 2:3) - traces.ref(end, 1:2)).';
        ctrl_perform = sqrt( min( max( model.u_min^2, (options.track_open_loop * ...
            open_loop_ctrl^2) + v_lqr ), model.u_max^2 ) );
    end
    
    % Saturate performance control BEFORE combining with safety control
    ctrl_perform = max( model.u_min, min( ctrl_perform, model.u_max ) );

    %% Get safety controller
    % Get optimal control used for reachability calculations
    deriv = eval_u(reachability.grid, reachability.gradient, ...
        [traces.state(end, 2) traces.state(end, 3)]);
    ctrl_safety = optCtrl(reachability.obj, 0, 0, deriv, model.uMode);

    %% Predict next state
    % y = y0 + vt + at^2 and v = v0 + at
    if size(traces.accel, 1) >= 1
        pred_state(1) = traces.state(end, 2) + traces.state(end, 3) * options.pred_time + ...
            traces.accel(end) * (options.pred_time)^2;
        pred_state(2) = traces.state(end, 3) +  traces.accel(end) * options.pred_time;
        predReachVal = interpn(reachability.grid.vs{1}, reachability.grid.vs{2}, ...
                reachability.valueFunc, pred_state(1), pred_state(2));
    else
        predReachVal = reachVal;
    end
    %% Calculate relative safety level
    % Compare your state and predicted next state to the safest state
    Vmin = min(reachability.valueFunc(:));
    reachValZero = traces.reachValZero(end) / 2; % dividing by 2 b/c it was too conservative
    %reachValZero = 0;
    eta = (reachValZero - reachVal) / (reachValZero - Vmin);
    pred_eta = (reachValZero - predReachVal) / (reachValZero - Vmin);
    min_eta = min(eta, pred_eta);
    beta = max( 0, min( (min_eta - options.eta_l) / (options.eta_u - options.eta_l), 1 ) );

    %% Determine control through combination of two controllers
    control = (ctrl_perform * beta) + (ctrl_safety * (1 - beta));

    % Record control (unless time has not changed)
    if size(traces.ctrl, 1) < 1 || data_time > traces.ctrl(end, 1)
        current_ctrl = [data_time, control];
        traces.ctrl = [traces.ctrl; current_ctrl];
    end
      
    %% Send control command 
    if options.dry_run
        % Update state based on dynamics of system and "real" disturbance at given state
        real_d_index = 1 + floor(100 * (traces.state(end, 2) - model.target(1, 1)));
        if real_d_index >=1 && real_d_index <= floor(2 + (model.target(2, 1) - model.target(1, 1)) * 100)
            next_state = updateState(reachability.obj, control, model.dt, ...
                [traces.state(end, 2) traces.state(end, 3)], [0 model.real_d(real_d_index)]);
        end
    else
        % Send command to specify control
    end
        
    %% Record current state 
    % Record state (unless time has not changed)
    if size(traces.state, 1) < 1 || data_time > traces.state(end, 1)
        state = [data_time, next_state(1), next_state(2)];
        traces.state = [traces.state; state];
    end
    
    %% Update time
    current_time = now;
end

fprintf('Finished %.2f seconds of test!\n', options.test_run_duration);