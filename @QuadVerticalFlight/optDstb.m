function dOpt = optDstb(obj, ~, ~, deriv, dMode)
% dOpt = optCtrl(obj, t, y, deriv, dMode)

%% Input processing
if nargin < 5
  dMode = 'max';
end

if ~iscell(deriv)
  deriv = num2cell(deriv);
end

dOpt = cell(obj.nd, 1);

%% Optimal disturbance
d_index = find(int32(100 * obj.dMax(:, 1)) == int32(100 * obj.x(1)));

if strcmp(dMode, 'max')
  for i = 1:2
    if any(obj.dims == i)
      dOpt{i} = (deriv{obj.dims==i}>=0)*(obj.dMax(d_index(1), i+1)) + ...
        (deriv{obj.dims==i}<0)*(obj.dMin(d_index(1), i+1));
    end
  end

elseif strcmp(dMode, 'min')
  for i = 1:2
    if any(obj.dims == i)
      dOpt{i} = (deriv{obj.dims==i}>=0)*(obj.dMin(d_index, i+1)) + ...
        (deriv{obj.dims==i}<0)*(obj.dMax(d_index, i+1));
    end
  end
else
  error('Unknown dMode!')
end

end