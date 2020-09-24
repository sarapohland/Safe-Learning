function dx = dynamics(obj, ~, x, u, d)
% Dynamics of Drunk
%    \dot{x}_1 = u + d
%    \dot{x}_2 = 0.5 * u + x_1
%   Control: u

if nargin < 5
  d = [0; 0];
end

if iscell(x)
  dx = cell(length(obj.dims), 1);
  
  for i = 1:length(obj.dims)
    dx{i} = dynamics_cell_helper(obj, x, u, d, obj.dims, obj.dims(i));
  end
else
  dx = zeros(obj.nx, 1);
  
  dx(1) = u + d(1);
  dx(2) = 0.5 * u + x(1) + d(2);
end
end

function dx = dynamics_cell_helper(obj, x, u, d, dims, dim)

switch dim
  case 1
    dx = u + d{1};
  case 2
    dx = 0.5 * u + x{dims==1} + d{2};
  otherwise
    error('Only dimension 1-2 are defined for dynamics of Drunk!')
end
end