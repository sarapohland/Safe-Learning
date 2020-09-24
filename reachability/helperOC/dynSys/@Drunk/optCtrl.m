function uOpt = optCtrl(obj, ~, ~, deriv, uMode)
% uOpt = optCtrl(obj, t, y, deriv, uMode)

%% Input processing
if nargin < 6
  uMode = 'min';
end

if ~iscell(deriv)
  deriv = num2cell(deriv);
end

%% Optimal control
ctrlCond = deriv{obj.dims==1} + 0.5 * deriv{obj.dims==2};
if strcmp(uMode, 'max')
  uOpt = (ctrlCond>=0)*(obj.uMax) + (ctrlCond<0)*(obj.uMin);
elseif strcmp(uMode, 'min')
  uOpt = (ctrlCond>=0)*(obj.uMin) + (ctrlCond<0)*(obj.uMax);
else
  error('Unknown uMode!')
end

end