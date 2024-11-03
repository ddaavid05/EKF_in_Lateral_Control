% Define the time span and initial condition
% tspan = [0 5];
% y0 = 1;
% 
% Solve the differential equation
% [t, y] = ode45(@simpleODE, tspan, y0);
% 
% Display the results
% disp('Time points:');
% disp(t);
% disp('Solution values:');
% disp(y);
% 
% Plot the results
% plot(t, y)
% xlabel('Time t')
% ylabel('Solution y')
% title('Solution of dy/dt = -2y')


% Parameters
Cf=80000;
Cr=80000;
lf=1.4;
lr=1.8;
m=1500;
Iz=1200;

% More Params
T = 0.01;
Vx = 20;
x_Prior = [4 5 6 7]';
control_vector = [0.17 1]';


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
x0 = x_Prior; 

% Define input control vector (u)
u = control_vector;

% Initialize t and x for clarity 
% t = []; 
% x = [];

% Solve the state equation using ode45 
[t, x] = ode45(@(t, x) Ac*x + Bc*u, tspan, x0);


% Predicted state (x_pred) 
x_pred = x(end,:)'

% Display the results
% disp('Time points:');
% disp(t);
% disp('Solution values:');
% disp(x);

% Plot the results
plot(t, x)
xlabel('Time t')
ylabel('Solution x')
title('Solution of x_dot')