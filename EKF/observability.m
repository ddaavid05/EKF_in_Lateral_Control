% Parameters
Cf=80000;
Cr=80000;
lf=1.4;
lr=1.8;
m=1500;
Iz=1200;
% More params
Vx = 1.5;

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
  
H = [ 1, 0, 0, 0;
      0, 0, 1, 0];
  
O = [H; 
     H*Ac;
     H*(Ac)^2;
     H*Ac^3];

rank(O)


% Convert continuous-time to discrete-time 
sysc = ss(Ac, Bc, [], []); 
sysd = c2d(sysc, T); 
Ad = sysd.A
Bd = sysd.B