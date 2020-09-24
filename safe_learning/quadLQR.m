function K = quadLQR(model, options)
% This function calculates the LQR parameters for the quadcopter vertical
% flight.

% Control matrics
Q = options.Q(1:2, 1:2);
R = options.R(1);

% Continuous-time model with dynamics dx/dt = Ax + Bu
A = [0 1; 0 0];
B = [0; model.k_T];

% Calculate optimal gain matrix K
try
    K = lqr(A, B, Q, R);
catch
    ME = MException('LQR_bad', ...
             'System is not stabilizable or input cost is zero.');
    throw(ME)
end

end