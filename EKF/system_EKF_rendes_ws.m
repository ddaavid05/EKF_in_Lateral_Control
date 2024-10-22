% Parameters
Cf=80000;
Cr=80000;
lf=1.4;
lr=1.8;
m=1500;
Iz=1200;

% Sampling time
T = 0.01;

e1_noise_std = 0.05; % 5 cm
e2_noise_std = deg2rad(0.5); % 0.5 degree

% Measurement noise covariance matrix (constant)
R = diag([e1_noise_std^2, e2_noise_std^2]);

% Process noise covariance
sigma_e1_dot = 0.1;
sigma_e2_dot = 0.05;
Q_e1 = sigma_e1_dot^2 * [T^2 T 0 0;
                         T  1 0 0;
                         0  0 0 0;
                         0  0 0 0];

Q_e2 = sigma_e2_dot^2 * [0 0 0 0;
                         0 0 0 0;
                         0 0 T^2 T;
                         0 0 T 1];

Q = Q_e1 + Q_e2;

P0 = eye(4) * 10;
X0 = zeros(4, 1);

% Load simulink
load_system('system_EKF_rendes')
open_system('system_EKF_rendes')
sim('system_EKF_rendes')