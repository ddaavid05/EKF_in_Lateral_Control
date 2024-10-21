% Parameters
Cf = 80000;
Cr = 80000;
lf = 1.4;
lr = 1.8;
m = 1500;
Iz = 1200;

% Dynamic parameter
Vx = 20;
delta = 1;
yaw_rate_desired = 1;

% State space model elements
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

% Sampling time
T = 0.1;

% Discretize the continuous-time state-space system
sysC = ss(Ac, Bc, [], []);
sysD = c2d(sysC, T);

Ad = sysD.A;
Bd = sysD.B;

% Initial state and covariance
x = [0; 0; 0; 0]; % Initial state
P = eye(4);       % Initial state covariance

% Process and measurement noise covariances
Q = eye(4) * 0.1; % Process noise covariance
R = eye(2) * 0.1; % Measurement noise covariance

% Control input
u = [delta; yaw_rate_desired];

% Time-update (prediction) step
x_pred = Ad * x + Bd * u;
% Jacobian of the state transition function
F_x = Ad;
P_pred = F_x * P * F_x' + Q;

% Measurement matrix for measuring only e1 and e2
C = [1 0 0 0;
     0 0 1 0];

% Measurement update step
K = P_pred * C' / (C * P_pred * C' + R);
z = [e1_measured; e2_measured]; % Measurement vector (replace with actual measurements)
x = x_pred + K * (z - C * x_pred);
P = (eye(4) - K * C) * P_pred;

% Display results
disp('Predicted state:');
disp(x_pred);
disp('Updated state:');
disp(x);
