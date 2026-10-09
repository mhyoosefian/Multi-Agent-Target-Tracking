function [new_pos, new_rot] = follower_dynamics(old_pose, old_psi, u, Ts, k)

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
d = 0.1*sin(0.4*k) + 0.1*cos(0.4*k);
new_pos = Ts*(old_rot*lin_vel) + old_pose;