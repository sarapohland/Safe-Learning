function [model, reachability] = updateReachSet(model, gp,...
    valueFuncCurrent)
% This function uses the model of disturbance created using Gaussian
% Processes to perform updated reachability analysis.

% Maximum disturbance is now the value 2 standard deviations above the mean
% and minimum disturbance is 2 standard deviations below the mean for every
% altitude that was used as a test input in the GP computations
model.d_max = [gp.test_inputs, gp.mean + 2*sqrt(gp.std_dev)];
model.d_min = [gp.test_inputs, gp.mean - 2 * sqrt(gp.std_dev)];

[reachability.valueFunc, reachability.grid] = calcReachSet(model, valueFuncCurrent);
reachability.gradient = computeGradients(reachability.grid, reachability.valueFunc);
reachability.obj = QuadVerticalFlight(model, 1:2);
end

