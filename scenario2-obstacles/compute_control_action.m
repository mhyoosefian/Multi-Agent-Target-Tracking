function u = compute_control_action(eps, psi, pos, des_pos, des_omega, Kp, gamma, gamma0, rx, ry, target_vel, act_lim)

% Extracting actuator limits
min_lin_vel = act_lim(1, 1);
min_ang_vel = act_lim(1, 2);
max_lin_vel = act_lim(2, 1);
max_ang_vel = act_lim(2, 2);

% Computing Delta bar matrix
Delta = [1  eps(2)
         0 -eps(1)];
Delta_bar = Delta'*inv(Delta*Delta');

% Computing position tracking error
rot = [cos(psi) -sin(psi)
           sin(psi)  cos(psi)];
e_p = rot'*(pos - des_pos) - eps;

% Computing derivative of r (gamma0 is not necessarily zero)
r_prime = [-rx*sin(gamma + gamma0)
           ry*cos(gamma + gamma0)];

% Computing the control action
u = Delta_bar*(rot'*(r_prime*des_omega + target_vel) - Kp*e_p);

% Checking saturation
if u(1) <= min_lin_vel
    u(1) = min_lin_vel;
end
if u(1) >= max_lin_vel
    u(1) = max_lin_vel;
end
if u(2) <= min_ang_vel
    u(2) = min_ang_vel;
end
if u(2) >= max_ang_vel
    u(2) = max_ang_vel;
end