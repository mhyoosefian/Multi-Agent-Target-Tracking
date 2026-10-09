function [new_pos, new_rot, new_d] = follower_dynamics(old_pose, old_psi, u, Ts, C, A, old_d)

% Forming the old rotation matrix
old_rot = [cos(old_psi) -sin(old_psi)
           sin(old_psi)  cos(old_psi)];

% Extracting velocities from input
lin_vel = [u(1); 0];
rot_vel = u(2);

% Computing the new rotation matrix
S = [0 -rot_vel
     rot_vel 0];
new_rot = Ts*(old_rot*S) + old_rot;

% Computing the new position vector
new_pos = Ts*(old_rot*lin_vel + C*old_d) + old_pose;

% Computing the new disturbance
new_d = old_d + Ts*(A*old_d);

% Important note: The value of Ts matters. It should be really small. Since
% otherwise the discritization will be inaccurate and the disturbance
% controller will compensate will be different from this one.
