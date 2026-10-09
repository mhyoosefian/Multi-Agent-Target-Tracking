function out = cbf_controller2(pos, posd, des_omega, upre, no, xo, yo, ro, umin, umax, eps, Kp, gamma, gamma0, rx, ry, target_vel, l, agent_radius, num_static_obstacles)

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

% u_no = Delta_bar*(rot'*(r_prime*des_omega + target_vel) - Kp*e_p);
a = Delta_bar*(rot'*(r_prime*des_omega + target_vel));
E_no = [0 0 -Delta_bar(1, 1)*e_p(1) -Delta_bar(1, 2)*e_p(2) a(1);
        0 0 -Delta_bar(2, 1)*e_p(1) -Delta_bar(2, 2)*e_p(2) a(2)];
E_u = [eye(2) zeros(2, 3)];
E_kp1 = [0 0 1 0 0];
E_kp2 = [0 0 0 1 0];
E_kp = [E_kp1; E_kp2];

W1 = 10000*seye(2);
W2 = 0.0001*eye(2);

r_ro = agent_radius + ro;

B = zeros(no, 1);
LfB = zeros(no, 1);
LgB = zeros(no, 2);

for i = 1:no
    B(i) = (xp + l * cos(theta) - xo(i)) ^ 2 + (yp + l * sin(theta) - yo(i)) ^ 2 - r_ro(i) ^ 2;
    LgB(i, :) = [2 * (xp + l * cos(theta) - xo(i)) * cos(theta) + 2 * (yp + l * sin(theta) - yo(i)) * sin(theta), -2 * (xp + l * cos(theta) - xo(i)) * l * sin(theta) + 2 * (yp + l * sin(theta) - yo(i)) * l * cos(theta)];
end

lambda_B = .1;
alpha = 1.5;
beta = 0.5;


H = E_u'*W1*E_u - E_kp'*W2*E_kp;

f = zeros(5, 1);
A = - LgB;
A = [A zeros(no, 3)];
A = [A; -E_kp; E_u-alpha*E_no; -E_u+beta*E_no];
b = lambda_B * B + LfB;
b = [b; 0; 0; 0; 0; 0; 0];

LB = [umin; 0; 0; 1];
UB = [umax; 100; 100; 1];
Aeq = [0 0 0 0 1];
beq = 1;

X0 = [upre; Kp(1, 1); Kp(2, 2); 1];
[X, ~] = quadprog(H,f,A,b,Aeq,beq,LB,UB,X0);
u = X(1:2, 1);
kp1 = X(3, 1);
kp2 = X(4, 1);

% IMPORTANT: The order of the obstacles in xo and yo is important. 
% const_violation = zeros(num_static_obstacles, 1);
% for i=1:num_static_obstacles
%     const_violation(i) = A(i, :)*u - b(i); % The more negative this number is, the far the constraint is from violation. So, if it is close to zero, then it is
%                                         % close to violation
% end
out = [u; kp1; kp2];