function gaussian = modelDisturb(traces, model)
% This function uses Gaussian Processes (GP) to create a model of the
% disturbance. It receives training data and computes the mean and standard
% deviation for each test input. It also generates a graph to visualize the
% model and confidence interval.

fprintf('Performing GP calculations...\n');

%% Specify data
training_input = traces.state(4:10:end, 2); % altitudes
training_output = traces.filtered_disturb(1:10:end); % filtered disturbance
min_alt = model.target(1, 1);
max_alt = model.target(2, 1);
test_inputs = linspace(min_alt, max_alt, model.length_disturb_model)';

%% Specify mean, covariance, and likelihood functions
meanfunc = []; % empty: don't use a mean function
covfunc = @covSEiso; % Squared Exponental covariance function
likfunc = @likGauss; % Gaussian likelihood

%% Initialize hyperparameter struct
hyp = struct('mean', [], 'cov', [0 0], 'lik', log(0.1));
hyp2 = minimize(hyp, @gp, -100, @infGaussLik, meanfunc, covfunc, likfunc, training_input, training_output);

%% Make predictions using hyperparameters
[mu, s2] = gp(hyp2, @infGaussLik, meanfunc, covfunc, likfunc, training_input, training_output, test_inputs);

%% Plot precitive mean at test points
% Using 95% confidence bounds
figure(4); 
f = [mu + 2*sqrt(s2); flipdim(mu - 2 * sqrt(s2), 1)];
fill([test_inputs; flipdim(test_inputs,1)], f, [7 7 7]/8)
hold on; 
plot(test_inputs, mu, 'b-'); plot(training_input, training_output, 'r+')
min_data = min(training_output);
max_data = max(training_output);
min_int = min(mu - 2 * sqrt(s2));
max_int = max(mu + 2*sqrt(s2));
min_disturb = min(min_data, min_int);
max_disturb = max(max_data, max_int);
axis([min_alt max_alt min_disturb max_disturb]);
xlabel('Altitude');
ylabel('Disturbance');
title('Predicted Model of Disturbance');

altitudes = model.target(1, 1):0.01:model.target(2, 1);
plot(altitudes, model.real_d, 'm-', 'LineWidth',2);

hold off;

%% Output inmportant components
gaussian.test_inputs = test_inputs;
gaussian.std_dev = s2;
gaussian.mean = mu;

end

