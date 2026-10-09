function [l_state_hat_new, P_new] = estimate_leader_state_one(y, Ts, l_state_hat_old, P_old, f_pos, Q, R)

% The prediction step
F = [eye(2) Ts*eye(2)
     zeros(2) eye(2)];
l_state_hat_new = F*l_state_hat_old;
Q_discrete = Ts^2*Q;
P_new = F*P_old*F' + Q_discrete;

% The update step. Do it when we have a measurement.
if ~isempty(y)
    q1 = l_state_hat_new(1);
    q2 = l_state_hat_new(2);
    p1 = f_pos(1);
    p2 = f_pos(2);
    H = zeros(1, 4);
    H(1, 1) = -(p1-q1)/y;
    H(1, 2) = -(p2-q2)/y;

    y_estimate = norm(l_state_hat_new(1:2) - f_pos, 2);
    y_tilde = y - y_estimate;

    S = H*P_new*H' + R;
    K = P_new*H'*inv(S);
    l_state_hat_new = l_state_hat_new + K*y_tilde;
    P_new = (eye(4) - K*H)*P_new;
end

out = {};
out{1} = l_state_hat_new;
l_state_hat_new = out;
out = {};
out{1} = P_new;
P_new = out;



