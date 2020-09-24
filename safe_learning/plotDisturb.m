function plotDisturb(traces)
% This function can be used to plot the observed and filtered disturbance
% at various altitudes.

figure(2);
plot(traces.disturb(1:10:end, 2), traces.state(4:10:end, 2), 'r.')
title('Disturbance at Various Altitudes');
xlabel('Disturbance (m/s^2)');
ylabel('Altitude (m)');

figure(3);
plot(traces.filtered_disturb(1:10:end), traces.state(4:10:end, 2), 'r.')
title('Filtered Disturbance at Various Altitudes');
xlabel('Filtered Disturbance (m/s^2)');
ylabel('Altitude (m)');

end

