% Parameters
Cf=Vehicle_CorneringStiffnessFrontAxle.Value;
Cr=80000;
lf=1.4;
lr=1.8;
m=1500;
Iz=1200;

vehicleParams = struct('Cf', Cf, 'Cr', Cr, 'lf', lf, 'lr', lr, 'm', m, 'Iz', Iz);

% Create a Bus object
elems(1) = Simulink.BusElement;
elems(1).Name = 'Cf';
elems(1).DataType = 'double';

elems(2) = Simulink.BusElement;
elems(2).Name = 'Cr';
elems(2).DataType = 'double';

elems(3) = Simulink.BusElement;
elems(3).Name = 'lf';
elems(3).DataType = 'double';

elems(4) = Simulink.BusElement;
elems(4).Name = 'lr';
elems(4).DataType = 'double';

elems(5) = Simulink.BusElement;
elems(5).Name = 'm';
elems(5).DataType = 'double';

elems(6) = Simulink.BusElement;
elems(6).Name = 'Iz';
elems(6).DataType = 'double';

vehicleParamsBus = Simulink.Bus;
vehicleParamsBus.Elements = elems;


% Sampling time
T = 0.01;

% Treshold
treshold = 0.01;

% Noise
e1_noise_std = 0.05; % 5 cm
e2_noise_std = deg2rad(0.5); % 0.5 degree

% Measurement noise covariance matrix (constant)
R = diag([e1_noise_std^2, e2_noise_std^2]);

% Process noise covariance
sigma_e1_dot = 0.01;
sigma_e2_dot = 0.005;
Q_e1 = sigma_e1_dot^2 * [1 0 0 0;
                         0 1 0 0;
                         0 0 0 0;
                         0 0 0 0];

Q_e2 = sigma_e2_dot^2 * [0 0 0 0;
                         0 0 0 0;
                         0 0 1 0;
                         0 0 0 1];

Q = Q_e1 + Q_e2;

P0 = eye(4) * 1000;
x0 = zeros(4, 1);

% Load simulink
load_system('system_EKF_rendes_ACEKF');
open_system('system_EKF_rendes_ACEKF');
sim('system_EKF_rendes_ACEKF');