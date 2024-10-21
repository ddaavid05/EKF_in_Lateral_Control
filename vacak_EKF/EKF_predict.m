function [x_pred, P_pred] = EKF_predict(input)

    % Parameters
    Cf = 80000;
    Cr = 80000;
    lf = 1.4;
    lr = 1.8;
    m = 1500;
    Iz = 1200;

    % Dynamic parameter
    Vx = input(5); % Now Vx is part of input

    % Define symbolic variables for states and controls
    syms e1 e1_dot e2 e2_dot delta yaw_rate_desired Vx

    % State vector
    state = [e1; e1_dot; e2; e2_dot];

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
    Jacobian_F = jacobian(F, state);

    % Convert symbolic Jacobian to numerical function handle
    Jacobian_F_func = matlabFunction(Jacobian_F, 'Vars', {state, delta, yaw_rate_desired, Vx});

    % Extract current state and covariance
    x = input(1:4);  % Initial state
    P = reshape(input(6:end), 4, 4); % Initial state covariance
    
    % Process noise covariance
    Q = eye(4) * 0.1; % Adjust as necessary
    I = eye(4); % Identity matrix

    % Sampling period
    T = 0.1;

    % Time-update (prediction) step
    F_x = Jacobian_F_func(x(1), x(2), x(3), x(4), delta, yaw_rate_desired, Vx);
    x_pred = x + T * double(subs(F, {e1, e1_dot, e2, e2_dot, delta, yaw_rate_desired, Vx}, num2cell([x; delta; yaw_rate_desired; Vx])));
    P_pred = (I + T * F_x) * P * (I + T * F_x)' + T * Q;
end
