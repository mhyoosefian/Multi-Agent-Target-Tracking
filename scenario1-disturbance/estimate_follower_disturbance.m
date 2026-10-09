function [f_pos_new, f_psi_new, d_new, f_pos_hat_new, f_psi_hat_new, d_hat_new, P_f] = estimate_follower_disturbance(f_pos_old, f_psi_old, d_old, ...
    f_pos_hat_old, f_psi_hat_old, d_hat_old, u, sqrt_Q_f, sqrt_R_f, Q_f, R_f, P_f, Ts, p, m, n, Cx, Cy, A_d)

% Apply the control action to the followers
w_f = sqrt_Q_f*randn(p, 1);
v_f = sqrt_R_f*randn(m, 1);
X = f([f_pos_old; f_psi_old; d_old], u, w_f, Ts, Cx, Cy, A_d);
f_pos_new = X(1:2);
f_psi_new = X(3);
d_new = X(4:end);


% Propagation
F = jacob_f_X([f_pos_hat_old; f_psi_hat_old; d_hat_old], u, Ts);
G = jacob_f_w([f_pos_hat_old; f_psi_hat_old; d_hat_old], u, Ts);

Xh_minus_ekf = f([f_pos_hat_old; f_psi_hat_old; d_hat_old], u, zeros(p, 1), Ts, Cx, Cy, A_d);
P_minus_ekf = F*P_f*F.' + G*Q_f*G.';

% Observation (measurement)
y_f = h([f_pos_old; f_psi_old; d_old], v_f);

% Update
H = jacob_h_X(Xh_minus_ekf);
M = jacob_h_v(Xh_minus_ekf);
K = P_minus_ekf*H.'*inv(H*P_minus_ekf*H.' + M*R_f*M.');
Xh = Xh_minus_ekf + K*(y_f - h(Xh_minus_ekf, zeros(m, 1)));
f_pos_hat_new = Xh(1:2);
f_psi_hat_new = Xh(3);
d_hat_new = Xh(4:end);
P_f = (eye(n) - K*H)*P_minus_ekf;