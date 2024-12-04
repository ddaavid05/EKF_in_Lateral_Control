% Load data from the workspace
out = evalin('base', 'out');

% Extract time vectors from 'continuous_state' and selected structures
time_meas = out.measurementsEKF.Time;
time_pred = out.predictionsEKF.Time;

% Plot all data columns in a 2x2 layout
figure;
data_columns = size(out.measurementsEKF.Data, 2); % Number of data columns
h = gobjects(1, data_columns); % Initialize graphics object array for plots

for i = 1:data_columns
    % Switch-case to set unique y-labels
    switch i 
        case 1
            state_variable = 'Lateral position error'; 
        case 2
            state_variable = 'Lateral velocity error';
        case 3
            state_variable = 'Yaw error';
        case 4
            state_variable = 'Yaw rate error';
        otherwise
            state_variable = (['Data:', num2str(i)]);
    end
    
    h(i) = subplot(2, 2, i);
    plot(time_meas, out.measurementsEKF.Data(:, i), 'LineWidth', 1.5);
    hold on;
    plot(time_pred, out.predictionsEKF.Data(:, i), 'LineWidth', 1.5);
    hold off;
    ylim([-1, 1]);
    xlabel('Time (s)');
    ylabel(state_variable);
    legend('Measurement', "Estimated", 'Location', 'best');
    grid on;
end

% Plot longitudinal velocity in a separate figure (rectangular)
figure;
set(gcf, 'Position', [100, 100, 900, 300]); % Set figure size for rectangular shape
h_long_vel = gca;
plot(out.longSpeed.Time, out.longSpeed.Data, 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Longitudinal velocity (m/s)');
grid on;
% Link axes for synchronized zooming and panning (only x-axis)
linkaxes([h, h_long_vel], 'x'); % Link all axes in x direction
