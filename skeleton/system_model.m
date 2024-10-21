function output = system_model(input)

% Parameters
Cf=80000;
Cr=80000;
lf=1.4;
lr=1.8;
m=1500;
Iz=1200;
% Dynamic parameter
Vx = input(6);
delta = input(4);
yaw_rate_desired = input(5);

% State space
Ac=[0 1                            0                    0;
    0 -((2*Cf+2*Cr)/(m*Vx))        (2*Cf+2*Cr)/m        (-2*Cf*lf+2*Cr*lr)/(m*Vx);
    0 0                            0                    1;
    0 -((2*Cf*lf-2*Cr*lr)/(Iz*Vx)) (2*Cf*lf-2*Cr*lr)/Iz -((2*Cf*lf^2+2*Cr*lr^2)/(Iz*Vx))]; 

Bc=[0            0;
    (2*Cf)/m     -((2*Cf*lf-2*Cr*lr)/(m*Vx));
    0            0;
    (2*Cf*lf)/Iz (-2*Cf*lf^2+2*Cr*lr^2)/(Iz*Vx)];

control_vector = [delta;
                  yaw_rate_desired];

% Discretisation
% Sampling time
T = 0.1;

% Create the continuous-time state-space system
sysC = ss(Ac, Bc, [], []);

% Discretize the system
sysD = c2d(sysC, T, 'zoh');

% Extract the discrete-time matrices
Ad = sysD.A;
Bd = sysD.B;

% Display the discrete-time matrices
disp('Countinous-time A matrix:');
disp(Ac);
disp('Countinous-time B matrix:');
disp(Bc);

disp('Discrete-time A matrix:');
disp(Ad);
disp('Discrete-time B matrix:');
disp(Bd);


output=Ad*input(1:4)+Bd*control_vector;


