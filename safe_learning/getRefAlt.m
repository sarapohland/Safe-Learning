function reference = getRefAlt(t, options)
% Generate a refernece trajectory for quadcopter to follow. It provides a
% reference altitude for an LQR controller to follow for a given time. 

% Sin: Desired trajectory follows sine function for all time
if strcmp(options.ref_mode, 'sin')
    ref_altitude = 0.8 + 0.8*sin(t/6);

% Smooth: Desired trajectory follows smooth path between 0.5 and 3 
elseif strcmp(options.ref_mode, 'smooth')
    t = mod(t,60);
    if t <= 5 || t > 55
        ref_altitude = 0.5;
    elseif t < 30
        ref_altitude = t/10;
    else
        ref_altitude = 3 - (t-30)/10;
    end

% Steps: Desired trajectory follows incremental path between 0.25 and 2.5 
elseif strcmp(options.ref_mode, 'steps')
    t = mod(t, 60);
    if t <= 5 || t > 55
        ref_altitude = 1.25;
    else
        t = mod(t,10);
        if t < 5
            ref_altitude = 2.5;
        else
            ref_altitude = 0.25;
        end
    end

% Training: Old training trajectory used to train disturbance model
elseif strcmp(options.ref_mode, 'training')
    t = mod(t, 60);
    
    if t <= 5
        ref_altitude = 1.25;
    elseif 5 < t && t <= 25
        ref_altitude = 1.25 + sin(2*pi/20*(t-5));
    elseif 25 < t && t <= 35
        ref_altitude = 1.25 + sin(2*pi/10*(t-25));
    elseif 35 < t && t <= 40
        ref_altitude = -0.21*t + 8.6;
    elseif 40 < t && t <= 45
        ref_altitude = 0.2;
    elseif 45 < t && t <= 50
        ref_altitude = 2.5;
    elseif 50 < t && t <= 55
        ref_altitude = 0.2;
    elseif 55 < t && t <= 60
        ref_altitude = 1.25;
    else
        ref_altitude = 1.25;
    end

% Testing: Desired trajectory varies between different altitudes outside of
% the safe region
elseif strcmp(options.ref_mode, 'testing')
    t = mod(t, 60);
    
    if t <= 5
        ref_altitude = 1.25;
    elseif 5 < t && t <= 10
        ref_altitude = -0.21*t + 2.3;
    elseif 10 < t && t <= 15
        ref_altitude = 0.2;
    elseif 15 < t && t <= 20
        ref_altitude = 3;
    elseif 20 < t && t <= 25
        ref_altitude = 0.4;
    elseif 25 < t && t <= 45
        ref_altitude = 1.25 + sin(2*pi/20*(t-10));
    elseif 45 < t && t <= 55
        ref_altitude = 1.25 + sin(2*pi/10*(t-25));
    elseif 55 < t && t <= 60
        ref_altitude = 1.25;
    else
        ref_altitude = 1.25;
    end

% Descent: Trajectory used in real-world demo of quadcopter learning about
% disturbance created by fan
elseif strcmp(options.ref_mode, 'descent')
    t = mod(t, 7);
    if t <= 1
        ref_altitude = 1.5;
    elseif t <= 4
        ref_altitude = 0.1;
    else
        ref_altitude = 1.5;
    end
    
% Train_New: New training trajectory that allows system to learn about its
% environment to create a disturbance model of the entire safe space
elseif strcmp(options.ref_mode, 'training_new')
    t = mod(t, 100);
    
    if t <= 5
        ref_altitude = 1.5;
    elseif 5 < t && t <= 10
        ref_altitude = 1.75;
    elseif 10 < t && t <= 15
        ref_altitude = 2;
    elseif 15 < t && t <= 20
        ref_altitude = 2.5;
    elseif 20 < t && t <= 25
        ref_altitude = 2.75;
    elseif 25 < t && t <= 30
        ref_altitude = 2.5;
    elseif 35 < t && t <= 40
        ref_altitude = 2.25;
    elseif 40 < t && t <= 45
        ref_altitude = 2.0;
    elseif 45 < t && t <= 50
        ref_altitude = 1.75;
    elseif 50 < t && t <= 55
        ref_altitude = 1.5;
    elseif 55 < t && t <= 60
        ref_altitude = 1.25;
    elseif 60 < t && t <= 65
        ref_altitude = 1.0;
    elseif 65 < t && t <= 70
        ref_altitude = 0.75;
    elseif 70 < t && t <= 75
        ref_altitude = 0.5;
    elseif 75 < t && t <= 80
        ref_altitude = 0.25;
    elseif 80 < t && t <= 85
        ref_altitude = 0.5;
    elseif 85 < t && t <= 90
        ref_altitude = 1.0;
    elseif 90 < t && t <= 95
        ref_altitude = 1.25;
    elseif 95 < t && t <= 100
        ref_altitude = 1.5;
    else
        ref_altitude = 1.5;
    end 
    
else
    ref_altitude = 1.25;
end

reference = [ref_altitude, 0, 0];

end