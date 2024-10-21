% Parameters
Cf = 80000;
Cr = 80000;
lf = 1.4;
lr = 1.8;
m = 1500;
Iz = 1200;

% Dynamic parameter
Vx = 20;
% delta = 1;
% yaw_rate_desired = 1;

% Define symbolic variables for states and controls
syms e1 e1_dot e2 e2_dot delta yaw_rate_desired

% State vector
x = [e1; e1_dot; e2; e2_dot];

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

% Nonlinear state transition function
F = Ac * x + Bc * [delta; yaw_rate_desired];

% Calculate Jacobian matrix of F with respect to x
Jacobian_F = jacobian(F, x);

% Display Jacobian matrix
disp('Jacobian matrix F_x:');
disp(Jacobian_F);
