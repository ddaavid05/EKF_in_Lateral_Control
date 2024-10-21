% Parameters
Cf=80000;
Cr=80000;
lf=1.4;
lr=1.8;
m=1500;
Iz=1200;
% Dynamic parameter
Vx = 20;
delta = 1;
yaw_rate_desired = 1;

% Elements
P1 = -((2*Cf+2*Cr)/(m*Vx));
P2 = (2*Cf+2*Cr)/m;
P3 = (-2*Cf*lf+2*Cr*lr)/(m*Vx);
P4 = -((2*Cf*lf-2*Cr*lr)/(Iz*Vx));
P5 = (2*Cf*lf-2*Cr*lr)/Iz;
P6 = -((2*Cf*lf^2+2*Cr*lr^2)/(Iz*Vx));
P7 = (2*Cf)/m;
P8 = -((2*Cf*lf-2*Cr*lr)/(m*Vx))-Vx;
P9 = (2*Cf*lf)/Iz;
P10 = -(2*Cf*lf^2+2*Cr*lr^2)/(Iz*Vx);

% State space
Ac=[0 1  0  0;
    0 P1 P2 P3;
    0 0  0  1;
    0 P4 P5 P6]; 

Bc=[0  0;
    P7 P8;
    0  0;
    P9 P10];

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

% Calculated A_discret B_discret
A_discret = [1 T+(T^2/2)*P1 (T^2/2)*P2   (T^2/2)*P3;
             0 1+T*P1        T*P2         T*P3;
             0 (T^2/2)*P4    1+(T^2/2)*P5 T+(T^2/2)*P6;
             0 T*P4          T*P5         1+T*P6]; 
         
B_discret = [(T^2/2)*P7 (T^2/2)*P8;
              T*P7       T*P8;
             (T^2/2)*P9 (T^2/2)*P10;
              T*P9       T*P10];
          
% Extended calculated discrete matrixes: A_ext B_ext
A_ext = [1 T  (T^2)/2 0  0  0;
         0 1  T       0  0  0;
         0 P1 0       P2 P3 0;
         0 0  0       1  T  T^2/2;
         0 0  0       0  1  T;
         0 P4 0       P5 P6 0];
     
B_ext = [0  0;
         0  0;
         P7 P8;
         0  0;
         0  0;
         P9 P10];
     
% e^Ac*T form
k = 4; % Number of terms in the Taylor series
A_calc = expm_taylor(Ac, T, k);
% B_calc = calculateBdfromAd(A_calc, Ac, Bc);

A_calc_validation = eye(size(Ac)) + (1/factorial(1))*Ac*T + (1/factorial(2))*Ac^2*T^2 + (1/factorial(3))*Ac^3*T^3 + (1/factorial(4))*Ac^4*T^4 + (1/factorial(5))*Ac^5*T^5;

% Compute the discrete-time A matrix
% A_discrete = expm(A_continuous * T_s);
% Compute the integral part for B_discrete
% integral_part = Ac \ (expm(Ac * T) - eye(size(Ac)));
% % Compute the discrete-time B matrix
% B_discrete = integral_part * Bc;


function Ad = expm_taylor(Ac, T, maxIter)
    % X is the matrix for which to compute the exponential
    [n, m] = size(Ac);
    if n ~= m
        error('Input must be a square matrix');
    end
    % Initialize Ad as a zero matrix of the same size as Ac
    Ad = zeros(size(Ac)); 
    % First term in the series (X^0 / 0!)
    term = eye(n);
    for k = 1:(maxIter+1)
        Ad = Ad + term;
        term = Ac * T * term * (1/k); % Update the term for the next iteration
    end
end

function Bd = calculateBdfromAd(Ad, Ac, Bc)
    % Ensure A and Ac are square matrices
    [n, m] = size(Ad);
    [p, q] = size(Ac);
    if n ~= m
        error('Matrix Ad must be square');
    end
    if p ~= q
        error('Matrix Ac must be square');
    end

    % Identity matrix of the same size as Ad
    I = eye(n);

    % Compute the inverse of Ac
    Ac_inv = inv(Ac);
    % turn singular matrix warning to error
%     myWarn = warning('error','MATLAB:singularMatrix'); 
%     try
%         Ac_inv = mrdivide(I,Ac);
%     catch
%         Ac_inv = pinv(Ac);
%     end
%     % return to warning
%     warning(myWarn); 
    
    % Compute B
    Bd = (Ad - I) * Ac_inv * Bc;
end




% Display the discrete-time matrices
% disp('Countinous-time A matrix:');
% disp(Ac);
% disp('Countinous-time B matrix:');
% disp(Bc);
% 
% disp('Discrete-time c2d A matrix:');
% disp(Ad);
% disp('Discrete-time c2d B matrix:');
% disp(Bd);
% 
% disp('Discrete-time calculated A matrix:');
% disp(A_discret);
% disp('Discrete-time calculated B matrix:');
% disp(B_discret);



