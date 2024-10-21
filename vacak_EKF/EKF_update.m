function output = EKF_update(x_pred, P_pred, z)
    % Parameters
    syms e1 e1_dot e2 e2_dot

    % State vector
    state = [e1; e1_dot; e2; e2_dot];

    % Measurement function (example: measuring only position states)
    h = [e1; e2];

    % Calculate Jacobian matrix of h with respect to state
    Jacobian_H = jacobian(h, state);

    % Convert symbolic Jacobian to numerical function handle
    H_func = matlabFunction(Jacobian_H, 'Vars', {state});

    % Predicted measurement
    h_pred = double(subs(h, {e1, e2}, {x_pred(1), x_pred(3)}));

    % Measurement residual
    y = z - h_pred;

    % Jacobian of measurement function at x_pred
    H = double(subs(Jacobian_H, {e1, e1_dot, e2, e2_dot}, {x_pred(1), x_pred(2), x_pred(3), x_pred(4)}));

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
    
    output = [x_upd, P_upd];
end
