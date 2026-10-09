function out = cbf_controller(pos, posd, des_omega, upre, no, xo, yo, ro, umin, umax, eps, Kp, gamma, gamma0, rx, ry, target_vel, l, agent_radius, num_static_obstacles)

% Note: no is the total number of obstacles, while num_static_obstacles is
% the number of static obstacles (da!)
theta = pos(3);
xp = pos(1) - l*cos(theta);
yp = pos(2) - l*sin(theta);

Delta = [1 eps(2)
         0 -eps(1)];

Delta_bar = Delta'*inv(Delta*Delta');

rot = [cos(theta) -sin(theta)
       sin(theta) cos(theta)];
e_p = rot'*(pos(1:2) - posd) - eps;

r_prime = [-rx*sin(gamma + gamma0)
           ry*cos(gamma + gamma0)];

u_no = Delta_bar*(rot'*(r_prime*des_omega + target_vel) - Kp*e_p);

r_ro = agent_radius + ro;

B = zeros(no, 1);
LfB = zeros(no, 1);
LgB = zeros(no, 2);

for i = 1:no
    B(i) = (xp + l * cos(theta) - xo(i)) ^ 2 + (yp + l * sin(theta) - yo(i)) ^ 2 - r_ro(i) ^ 2;
    LgB(i, :) = [2 * (xp + l * cos(theta) - xo(i)) * cos(theta) + 2 * (yp + l * sin(theta) - yo(i)) * sin(theta), -2 * (xp + l * cos(theta) - xo(i)) * l * sin(theta) + 2 * (yp + l * sin(theta) - yo(i)) * l * cos(theta)];
end

lambda_B = 0.1;

H = eye(2);
f = (- 2 * u_no' * H)';
A = - LgB;
b = lambda_B * B + LfB;
LB = umin;
UB = umax;
Aeq = [];
beq = [];

X0 = upre;
[u, ~] = quadprog(H,f,A,b,Aeq,beq,LB,UB,X0);

% IMPORTANT: The order of the obstacles in xo and yo is important. 
const_violation = zeros(num_static_obstacles, 1);
for i=1:num_static_obstacles
    const_violation(i) = A(i, :)*u - b(i); % The more negative this number is, the far the constraint is from violation. So, if it is close to zero, then it is
                                        % close to violation
end
out = [u; const_violation];