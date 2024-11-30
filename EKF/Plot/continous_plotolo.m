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

% Plot the state variables in separate figures and connect the predicted points to initial points

% State x1
figure;
plot(time_data, data1, 'g-', 'LineWidth', 1.5); % Plot the data from workspace
hold on;
plot([0, T], [x_prior(1), x_pred(1)], 'xr-', 'LineWidth', 1.5); 
xlabel('Time (t)');
ylabel('x1');
legend('Continous', 'First order Taylor');
grid on;
hold off;


% State x2
figure;
plot(time_data, data2, 'g-', 'LineWidth', 1.5); % Plot the data from workspace
hold on;
plot([0, T], [x_prior(1), x_pred(1)], 'xr-', 'LineWidth', 1.5); 
xlabel('Time (t)');
ylabel('x2');
legend('Continous', 'First order Taylor');
grid on;
hold off;
 
% State x3
figure;
plot(time_data, data3, 'g-', 'LineWidth', 1.5); % Plot the data from workspace
hold on;
plot([0, T], [x_prior(1), x_pred(1)], 'xr-', 'LineWidth', 1.5); 
xlabel('Time (t)');
ylabel('x3');
legend('Continous', 'First order Taylor');
grid on;
hold off;

% State x1
figure;
plot(time_data, data4, 'g-', 'LineWidth', 1.5); % Plot the data from workspace
hold on;
plot([0, T], [x_prior(1), x_pred(1)], 'xr-', 'LineWidth', 1.5); 
xlabel('Time (t)');
ylabel('x4');
legend('Continous', 'First order Taylor');
grid on;
hold off;
