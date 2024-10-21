function output = EKF_full(input)

    % Parameters
    Cf = 80000;
    Cr = 80000;
    lf = 1.4;
    lr = 1.8;
    m = 1500;
    Iz = 1200;

    % Extract current state and covariance
    x = input(1:4);  % Initial state
    Vx = input(5); % Now Vx is part of input
    z = input(6:7); % Measurement input
    P = reshape(input(8:23), 4, 4); 
    

    % State vector
    state = [x(1); x(2); x(3); x(4)];
    
    % Control Vector
    delta = z(1);
    yaw_rate_desired = z(2);

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
    F = Ac * state + Bc * [delta; yaw_rate_desired];

    % Calculate Jacobian matrix of F with respect to x
    Jacobian_F = [0 1 0 0;
                  0 P1 P2 P3;
                  0 0 0 1;
                  0 P4 P5 P6];
    
    % Process noise covariance
    Q = eye(4) * 0.1; % Adjust as necessary
    I = eye(4); % Identity matrix

    % Sampling period
    T = 0.1;

    % Time-update (prediction) step
    x_pred = x + T * F;
    P_pred = (I + T * Jacobian_F) * P * (I + T * Jacobian_F)' + T * Q;
    
    
    %%%%%%%%%%%%%%%%%%%%%%
    % Update
    % Measurement function (example: measuring only position states)
    h = [x_pred(1); x_pred(3)];

    % Calculate Jacobian matrix of h with respect to state
    Jacobian_H = [ 1, 0, 0, 0;
                   0, 0, 1, 0];

    % Measurement residual
    y = z - h;

    % Jacobian of measurement function at x_pred
    H = Jacobian_H;

    % Measurement noise covariance matrix (constant)
    R = eye(2) * 0.1;  % Adjust as necessary for your application

    % Innovation covariance
    S = H * P_pred * H' + R;

    % Optimal Kalman gain
    K = P_pred * H' / S;

    % Updated state estimate
    x_upd = x_pred + K * y;

    % Updated estimate covariance
    P_upd = (eye(size(P_pred)) - K * H) * P_pred;
    
    output = [x_upd; reshape(P_upd, [], 1)];


end