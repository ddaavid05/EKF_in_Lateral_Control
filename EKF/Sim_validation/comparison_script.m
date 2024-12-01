% Load data from the workspace
out = evalin('base', 'out');

% Create a 2x2 tiled layout for the error plots
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

% Save the 2x2 layout as an .svg file
[file1,path1] = uiputfile('*.svg','Save error plots as');
if ischar(file1) && ischar(path1)
    fullFileName1 = fullfile(path1, file1);
    saveas(gcf, fullFileName1);
end

% Create a separate plot for the velocity with rectangular shape
figure;
set(gcf, 'Position', [100, 100, 800, 200]); % Adjust the size to make it rectangular
plot(out.Vx.Time, out.Vx.Data, 'LineWidth', 1.5);
xlim([0, 5]);
xlabel('Time (s)');
ylabel('Velocity');
legend('Velocity', 'Location', 'best');
grid on;

% Save the velocity plot as an .svg file
% [file2,path2] = uiputfile('*.svg','Save velocity plot as');
% if ischar(file2) && ischar(path2)
%     fullFileName2 = fullfile(path2, file2);
%     saveas(gcf, fullFileName2);
% end
