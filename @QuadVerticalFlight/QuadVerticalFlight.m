classdef QuadVerticalFlight < DynSys
  properties
    uMax % Upper control bound
    uMin % Lower control bound
    dMax % Upper disturbance bound
    dMin % Lower disturbance bound
    g    % Gravitational acceleration
    kT   % Constant
    k0   % Constant
    dims % Dimensions that are active
  end
  
  methods
    function obj = QuadVerticalFlight(model, dims)
      % obj = QuadVerticalFlight(model, dims)
      %     Quadrotor Vertical Flight Class

      % Basic vehicle properties
      obj.pdim = [find(dims == 1) find(dims == 2)];
      obj.nx = length(dims);
      obj.nu = 1;
      obj.nd = 2;
      
      obj.x = [model.x1_init, model.x2_init];
      obj.xhist = obj.x;
      
      obj.uMax = model.u_max;
      obj.uMin = model.u_min;
      obj.dMax = [model.d_max(:, 1), zeros(length(model.d_max), 1), model.d_max(:, 2)];
      %obj.dMax = [0, model.d_max];
      obj.dMin = [model.d_max(:, 1), zeros(length(model.d_max), 1), model.d_min(:, 2)];
      %obj.dMin = [0, model.d_min];
      obj.g    = model.g;
      obj.kT   = model.k_T;
      obj.k0   = model.k_0;
      obj.dims = dims;
    end
    
  end % end methods
end % end classdef
