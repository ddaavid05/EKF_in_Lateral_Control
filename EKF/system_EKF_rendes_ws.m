% Parameters
Cf=80000;
Cr=80000;
lf=1.4;
lr=1.8;
m=1500;
Iz=1200;

% Sampling time
T = 0.05;

% Load simulink
load_system('system_EKF_rendes')
open_system('system_EKF_rendes')
sim('system_EKF_rendes')