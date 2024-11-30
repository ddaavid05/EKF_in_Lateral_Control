% Call the function with chosen methods
plot_methods({'Runge Kutta', 'First order Taylor', 'Continuous'});
% plot_methods({'First order Taylor', 'Continuous'});

function plot_methods(methods)
    % Parameters
    Cf = 80000;
    Cr = 80000;
    lf = 1.4;
    lr = 1.8;
    m = 1500;
    Iz = 1200;

    % More parameters
    T = 0.033;
    Vx = 40;
    x_prior = zeros(4, 1);
    control_vector = [0.1745329252; 1];

    % State-space model elements
    P1 = -((2*Cf + 2*Cr)/(m * Vx));
    P2 = (2*Cf + 2*Cr) / m;
    P3 = (-2*Cf * lf + 2*Cr * lr) / (m * Vx);
    P4 = -((2*Cf * lf - 2*Cr * lr) / (Iz * Vx));
    P5 = (2*Cf * lf - 2*Cr * lr) / Iz;
    P6 = -((2*Cf * lf^2 + 2*Cr * lr^2) / (Iz * Vx));
    P7 = (2*Cf) / m;
    P8 = -((2*Cf * lf - 2*Cr * lr) / (m * Vx)) - Vx;
    P9 = (2*Cf * lf) / Iz;
    P10 = -(2*Cf * lf^2 + 2*Cr * lr^2) / (Iz * Vx);

    % Continuous-time state-space matrices
    Ac = [0 1 0 0;
          0 P1 P2 P3;
          0 0 0 1;
          0 P4 P5 P6];

    Bc = [0 0;
          P7 P8;
          0 0;
          P9 P10];

    % Define the time span for integration
    tspan = [0 T]; % Integrate from 0 to T 

    % Initial conditions 
    x0 = x_prior; 

    % Define input control vector (u)
    u = control_vector;

    % Solve the state equation using ode45 
    [t, x_sol] = ode45(@(t, x) Ac*x + Bc*u, tspan, x0);

    % Discrete-time state-space matrices
    Ad = eye(size(Ac)) + Ac * T;
    Bd = Bc * T;

    % Time-update (prediction) step
    x_pred = Ad * x_prior + Bd * control_vector;

    % Load data from workspace
    out = load('C:\Users\Admin\Documents\Dávid\Egyetem\Mester\Diploma2\Matlab_representation\EKF\Plot\continous.mat');
    time_data = out.out.continous.Time;
    data1 = out.out.continous.Data(:, 1);
    data2 = out.out.continous.Data(:, 2);
    data3 = out.out.continous.Data(:, 3);
    data4 = out.out.continous.Data(:, 4);

    % Create figure
    figure;

    % Plot the state variables in a 2x2 layout and connect the predicted points to initial points, including workspace data
    % State x1
    subplot(2, 2, 1);
    hold on;
    if ismember('Runge Kutta', methods)
        plot(t, x_sol(:, 1), '-o', 'LineWidth', 1.5); % ODE45 solution
    end
    if ismember('First order Taylor', methods)
        plot([0, T], [x_prior(1), x_pred(1)], 'xr-', 'LineWidth', 1.5); % Predicted point
    end
    if ismember('Continuous', methods)
        plot(time_data, data1, 'g-', 'LineWidth', 1.5); % Workspace data
    end
    xlabel('Time (t)');
    ylabel('Lateral position error');
    legend(methods{:}, 'Location', 'best');
    grid on;
    hold off;

    % State x2
    subplot(2, 2, 2);
    hold on;
    if ismember('Runge Kutta', methods)
        plot(t, x_sol(:, 2), '-o', 'LineWidth', 1.5); % ODE45 solution
    end
    if ismember('First order Taylor', methods)
        plot([0, T], [x_prior(2), x_pred(2)], 'xr-', 'LineWidth', 1.5); % Predicted point
    end
    if ismember('Continuous', methods)
        plot(time_data, data2, 'g-', 'LineWidth', 1.5); % Workspace data
    end
    xlabel('Time (t)');
    ylabel('Lateral velocity error');
    legend(methods{:}, 'Location', 'best');
    grid on;
    hold off;

    % State x3
    subplot(2, 2, 3);
    hold on;
    if ismember('Runge Kutta', methods)
        plot(t, x_sol(:, 3), '-o', 'LineWidth', 1.5); % ODE45 solution
    end
    if ismember('First order Taylor', methods)
        plot([0, T], [x_prior(3), x_pred(3)], 'xr-', 'LineWidth', 1.5); % Predicted point
    end
    if ismember('Continuous', methods)
        plot(time_data, data3, 'g-', 'LineWidth', 1.5); % Workspace data
    end
    xlabel('Time (t)');
    ylabel('Yaw error');
    legend(methods{:}, 'Location', 'best');
    grid on;
    hold off;

    % State x4
    subplot(2, 2, 4);
    hold on;
    if ismember('Runge Kutta', methods)
        plot(t, x_sol(:, 4), '-o', 'LineWidth', 1.5); % ODE45 solution
    end
    if ismember('First order Taylor', methods)
        plot([0, T], [x_prior(4), x_pred(4)], 'xr-', 'LineWidth', 1.5); % Predicted point
    end
    if ismember('Continuous', methods)
        plot(time_data, data4, 'g-', 'LineWidth', 1.5); % Workspace data
    end
    xlabel('Time (t)');
    ylabel('Yaw-rate error');
    legend(methods{:}, 'Location', 'best');
    grid on;
    hold off;

    % Prompt user to select a save location
    [file, path] = uiputfile('*.svg', 'Save As');
    if ischar(file)
        % Save the figure as an SVG file
        saveas(gcf, fullfile(path, file), 'svg');
    else
        disp('File save operation canceled.');
    end
end
