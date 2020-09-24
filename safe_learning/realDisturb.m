function disturbance = realDisturb(model, options)
% This function creates a "real" disturbance to use for dry runs. 

% disturbance = 
% [lowest safe altitude    actual disturbance at altitude]
% [        ...                        ...                ]
% [highest safe altitude   actual disturbance at altitude]

num_points = floor(2 + (model.target(2, 1) - model.target(1, 1)) * 100);
disturbance = zeros(1, num_points);

% Linear: Disturbance varies linearly with altitude
% Minimum disturbance at lowest safe altitude and maximum at highest
if strcmp(options.real_d, 'linear')
    slope = (model.default_d_max - model.default_d_min) / (model.target(2, 1) - model.target(1, 1));
    y_intercept = model.default_d_max - (slope * model.target(2, 1));
    
    for i = 1 : num_points
        altitude = (i-1)/100 + model.target(1, 1);
        disturbance(i) = slope * altitude + y_intercept;
    end

% Fan: Disturbance is intended to reflect a calm room with fan on floor
% Disturbance is high near the ground and approaches zero as altitude
% increases
elseif strcmp(options.real_d, 'fan')
    for i = 1 : num_points
        altitude = (i-1)/100 + model.target(1, 1);
        disturbance(i) = 3 / (1 + exp(2 * altitude - 4));
    end  

% Ones: Disturbance is one everywhere
elseif strcmp(options.real_d, 'ones')
    disturbance = ones(1, num_points);

% Zeros: Disturbance is zero everywhere
elseif strcmp(options.real_d, 'zeros')
    disturbance = zeros(1, num_points);

% Threes: Disturbance is three everywhere
elseif strcmp(options.real_d, 'threes')
    disturbance = 3 * ones(1, num_points);
      
end
    

fprintf('Computed disturbance model!\n');
end