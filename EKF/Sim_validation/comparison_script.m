% Load data from the workspace
out = evalin('base', 'out');

% Create a 2x2 tiled layout for the plots
figure;
tiledlayout(2, 2);

% e1 comparison
nexttile;
plot(out.e1.Time, out.e1.Data(:, 1), 'LineWidth', 1.5);
hold on;
plot(out.e1.Time, out.e1.Data(:, 2), 'LineWidth', 1.5);
xlim([0, 5]);
hold off;
xlabel('Time (s)');
ylabel('Lateral position error');
legend('Vehicle state', 'Estimated state', 'Location', 'best');
grid on;

% e1_dot comparison
nexttile;
plot(out.e1_dot.Time, out.e1_dot.Data(:, 1), 'LineWidth', 1.5);
hold on;
plot(out.e1_dot.Time, out.e1_dot.Data(:, 2), 'LineWidth', 1.5);
xlim([0, 5]);
hold off;
xlabel('Time (s)');
ylabel('Lateral velocity error');
legend('Vehicle state', 'Estimated state', 'Location', 'best');
grid on;

% e2 comparison
nexttile;
plot(out.e2.Time, out.e2.Data(:, 1), 'LineWidth', 1.5);
hold on;
plot(out.e2.Time, out.e2.Data(:, 2), 'LineWidth', 1.5);
xlim([0, 5]);
hold off;
xlabel('Time (s)');
ylabel('Yaw error');
legend('Vehicle state', 'Estimated state', 'Location', 'best');
grid on;

% e2_dot comparison
nexttile;
plot(out.e2_dot.Time, out.e2_dot.Data(:, 1), 'LineWidth', 1.5);
hold on;
plot(out.e2_dot.Time, out.e2_dot.Data(:, 2), 'LineWidth', 1.5);
xlim([0, 5]);
hold off;
xlabel('Time (s)');
ylabel('Yaw-rate error');
legend('Vehicle state', 'Estimated state', 'Location', 'best');
grid on;

% Specify the location and name for the .svg file
[file,path] = uiputfile('*.svg','Save as');
if ischar(file) && ischar(path)
    fullFileName = fullfile(path, file);
    saveas(gcf, fullFileName);
end
