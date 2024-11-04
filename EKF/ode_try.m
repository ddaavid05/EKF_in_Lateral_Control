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

P_prior = [1 2 3 4;
           5 6 7 8;
           1 2 3 4;
           5 6 7 8];


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
% plot(t, x)
% xlabel('Time t')
% ylabel('Solution x')
% title('Solution of x_dot')


% Covariance
% Initial conditions 
P0 = P_prior; 

% Define Jacoby
F = Ac;

% Flatten the initial covariance matrix 
P0_vec = P0(:); 

% Solve the covariance equation using ode45 
[~, P_sol] = ode45(@(t, P_vec) covarianceODE(t, P_vec, F, Q), tspan, P0_vec); 

% Reshape the final solution back into a matrix 
P_pred = reshape(P_sol(end, :), 4, 4)

% Display the results
% disp('Time points:');
% disp(t);
disp('Solution P values:');
disp(P_sol);

% Plot the results
% plot(t, x)
% xlabel('Time t')
% ylabel('Solution x')
% title('Solution of x_dot')



% Define the function for the covariance 
function dPdt = covarianceODE(t, P_vec, F, Q) 
    P = reshape(P_vec, 4, 4); % Reshape the vector back into a matrix 
    dPdt = F*P + P*F' + Q; % Calculate the derivative 
    dPdt = dPdt(:); % Flatten the matrix back into a vector 
end