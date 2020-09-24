function model = modelParameters()
% This function defines inital and default parameters for the quadcopter
% vertical flight.

% Initial states for reachability computation
model.x1_init = 1.5;
model.x2_init = 0;

% Constants in dynamic model
model.g = -9.8;
model.k_T = 2.49;
model.k_0 = -6.34;

% Motor thrust control
model.u_min = 5;
model.u_max = 8;
model.u_hov = 6.5;

% Assumed (initial) disturbance
model.default_d_max = 2;
model.default_d_min = -2;

% Target set
% safe set: K = {x: 0.35m <= x1 <= 2.8m ; -3.5m/s <= x2 <= 3.5m/s}
model.target = [0.35, -3.5; 2.8,  3.5];

% Reachability grid
model.grid_count = 41;
model.grid_min = [-0.5 , -4];
model.grid_max = [3 , 4];

% Recahability time horizon
model.dt = 1/20;
model.t_max = 2; % originally 5s

% Reachability modes
% Goal: stay inside safe set
model.uMode = 'min';
model.dMode = 'max';

end


