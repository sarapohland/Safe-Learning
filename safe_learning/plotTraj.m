function plotTraj(traces)
% This function allows you to visualize the trajectory of the system and is
% intended to be used when performing a dry run.

hold on;
fprintf('Plotting trajectory (may take a little while)\n');

for i = 1:10:size(traces.state, 1)
    drawnow
    plot(traces.state(i, 2), traces.state(i, 3), 'r.')
end

% for i = 1:1:size(traces.ref, 1)
%     plot(traces.ref(i, 1), traces.ref(i, 2), 'ko', ...
%         'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'k', 'MarkerSize',5);
% end
hold off;
    
end