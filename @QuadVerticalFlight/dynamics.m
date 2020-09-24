function dx = dynamics(obj, ~, x, u, d)
% Define dynamics of quadrotor vertical flight
%   State:     x_1 = vehicle altitude
%              x_2 = vertical velocity 
%   Control:   u   = motor thrust command
%   Constants: g   = graviational acceleration
%              kT  = 
%              k0  =
%   Dynamics:  dot{x}_1 = x_2
%              dot{x}_2 = k_T * u + g + k_0 + d

% Define dynamic of system
if iscell(x)
  dx = cell(length(obj.dims), 1);
  
  for i = 1:length(obj.dims)
    dx{i} = dynamics_cell_helper(obj, x, u, d, obj.dims, obj.dims(i));
  end
else
  dx = zeros(obj.nx, 1);
  
  dx(1) = x(2);
  dx(2) = obj.kT * u + obj.g + obj.k0 + d(2);
end
end

function dx = dynamics_cell_helper(obj, x, u, d, dims, dim)
switch dim
  case 1
    dx = x{dims==2};
  case 2
    dx = obj.kT * u + obj.g + obj.k0 + d{2};
  otherwise
    error('Only dimension 1-2 are defined for dynamics of Quadrotor!')
end
end