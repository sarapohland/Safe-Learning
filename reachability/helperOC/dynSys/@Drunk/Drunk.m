classdef Drunk < DynSys
  properties
    uMax % Upper control bound
    uMin % Lower control bound
    dMax % Upper disturbance bound
    dMin % Lower disturbance bound
    dims % Dimensions that are active
  end
  
  methods
    function obj = Drunk(x, uMax, uMin, dMax, dMin, dims)
      % obj = Drunk(x, uMax, uMin, dMax, dMin, dims)
      %     Drunk class
      %
      % Dynamics:
      %    \dot{x}_1 = u + d
      %    \dot{x}_2 = 0.5 * u + x_1
      %         u \in [uMin, uMax]
      %         d \in [dMin, dMax]
      %
      % Inputs:
      %   x      - state: [xpos; ypos]
      %   uMax   - maximum forward velocity
      %   uMin   - minimum forward velocity
      %   dMax   - maximum disturbance
      %   dMin   - minimum disturbance
      %
      % Output:
      %   obj       - a Drunk object
      
      if numel(x) ~= obj.nx
        error('Initial state does not have right dimension!');
      end
      
      if ~iscolumn(x)
        x = x';
      end
      
      if nargin < 2
        uMax = 1;
      end
      
      if nargin < 3
        uMin = -1;
      end
      
      if nargin < 4
        dMax = [0; 0];
      end
      
      if nargin < 5
        dMin = [0; 0];
      end
      
      if nargin < 6
        dims = 1:2;
      end
      
      % Basic vehicle properties
      obj.pdim = [find(dims == 1) find(dims == 2)]; % Position dimensions
      obj.nx = length(dims);
      obj.nu = 1;
      obj.nd = 2;
      
      obj.x = x;
      obj.xhist = obj.x;
      
      obj.uMax = uMax;
      obj.uMin = uMin;
      obj.dMax = dMax;
      obj.dMin = dMin;
      obj.dims = dims;
    end
    
  end % end methods
end % end classdef
