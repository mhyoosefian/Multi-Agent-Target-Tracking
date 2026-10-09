% For three followers following one leader using only range information
% TO DO: Check the PFM. The gamma is not correct. check the initialization
% and everything. Check the codes of the paper.

close
clear
clc
rng(1);

set(0, 'DefaultTextInterpreter', 'latex');
set(0, 'DefaultAxesTickLabelInterpreter', 'latex');
set(0, 'DefaultLegendInterpreter', 'latex');
set(0, 'DefaultColorbarTickLabelInterpreter', 'latex');
set(0, 'DefaultAxesFontName', 'TimesNewRoman');
set(0, 'DefaultTextFontName', 'TimesNewRoman');
set(0, 'DefaultAxesFontSize', 8);
set(0, 'DefaultTextFontSize', 10);
set(0, 'DefaultLineLineWidth', 1);

%% General settings (includes all variables you might want to change)
show_animation = 0;
Ts = 0.1;               % sampling time
tf = 500;                % total simulation time
Tm = 0.1;
num_agents = 3;
use_default_initial_condition = 1;
obstacle_center{1} = [35; 45];
obstacle_radius{1} = 8;
obstacle_center{2} = [-20; 50];
obstacle_radius{2} = 10;
obstacle_center{3} = [-22; 0];
obstacle_radius{3} = 5;
% obstacle_center{4} = [3; 48];
% obstacle_radius{4} = 5;
num_static_obstacles = length(obstacle_center);
agent_radius = 3;
l = 0.2; % center to rear axle axis distance

% The adjacency matrix
% A = ones(num_agents);
A = [1 0 1; 1 1 0; 0 1 1];
% A = [1 0 0 0 1; 1 1 0 0 0; 0 1 1 0 0; 0 0 1 1 0; 0 0 0 1 1];

% Generating leader path
t = 0:Ts:tf;
q = [10*sin(0.02*t + pi)
     0.2*t];
Q = 0.00001*diag([1 1 1 1]);
for i=1:num_agents
    sigma{i} = 0.5;    % measurement noise std
end


% Desired values
omega_bar = 0.05;
rx = 30;
ry = 30;
eps = [-1; 0];
Kp = diag([1.5 .3]);
kc = 0.02;
max_lin_vel = 10;
min_lin_vel = -10;
max_ang_vel = 4;
min_ang_vel = -4;


% Estimation settings
if use_default_initial_condition == 1
    tmp_l_state_hat = {};
    tmp_P = {};
    for i=1:num_agents
        l_state_hat{i, 1} = [q(:, 1); -0.2; 0.3] + randn(4, 1);
        qhat{i} = l_state_hat{i, 1}(1:2);
        P{i, 1} = 0.1*(10 + 2*rand(1))*eye(4);
        tmp_l_state_hat{end + 1} = l_state_hat{i, 1};
        tmp_P{end + 1} = P{i, 1};
    end
end


%% Main code (Do not change anything in here)

% Noise settings
w = mvnrnd(zeros(4, 1), sqrtm(Q), tf/Ts)';
for i=1:num_agents
    R{i} = sigma{i}^2;
    v{i} = mvnrnd(0, sigma{i}, tf/Ts)';
end

% Initial desired locations
gamma = cell(num_agents, 1);
gamma{1}(1) = 0;
gamma0 = cell(num_agents, 1);
gamma0{1} = 0;
for i=2:num_agents
    gamma{i}(1) = gamma{i-1}(1) + 2*pi/num_agents;
    gamma0{i} = gamma0{i-1} + 2*pi/num_agents;
end
r = cell(num_agents, 1);
des_pos = cell(num_agents, 1);
for i=1:num_agents
    r{i} = [rx*cos(gamma{i}(1) + gamma0{i}); ry*sin(gamma{i}(1) + gamma0{i})];
    des_pos{i} = r{i} + q(:, 1);
end

% Initial conditions
f_psi = cell(num_agents, 1);
f_pos = cell(num_agents, tf/Ts+1);
l_state = [q(:, 1); -0.2; 0.2];
tmp_f_pos = {};
for i=1:num_agents
    f_psi{i} = 0;
    f_pos{i, 1} = des_pos{i};
    tmp_f_pos{end + 1} = f_pos{i, 1};
end

% Actuator limit
act_lim = [min_lin_vel min_ang_vel; max_lin_vel max_ang_vel];

% Main loop
k = 1;  % loop's counter
for t=0:Ts:tf-Ts

    if mod(k, 1000) == 0
        disp(k);
    end
    
    % Leader state evolution
    l_state(:, k+1) = [q(:, k+1); l_state(3:4, k)] +  w(:, k); %
    
    % Check if a measurement is received
    if mod(t, Tm) == 0
        tmp_y = {};
        for i=1:num_agents
            y{i, k} = norm(f_pos{i, k} - l_state(1:2, k), 2) + v{i}(k);
            tmp_y{end+1} = y{i, k};
        end
    else
       tmp_y = {};
       for i=1:num_agents
           y{i, k} = [];
           tmp_y{end+1} = y{i, k};
       end
    end

    % Estimate the state of the leader
    [tmp_l_state_hat_new, tmp_P_new] = estimate_leader_state(tmp_y, Ts, tmp_l_state_hat, tmp_P, tmp_f_pos, Q, R, A);
    tmp_l_state_hat = {};
    tmp_P = {};
    for i=1:num_agents
        P{i, k+1} = tmp_P_new{i};
        l_state_hat{i, k+1} = tmp_l_state_hat_new{i};
        qhat{i}(:, k+1) = l_state_hat{i, k+1}(1:2, :);
        tmp_l_state_hat{end + 1} = l_state_hat{i, k+1};
        tmp_P{end + 1} = P{i, k+1};
    end
    
    % Compute the desired trajectory for the follower
    tmp = cell2mat(gamma);
    for i=1:num_agents
        vc{i} = -kc*(sum(A(i, :))*gamma{i}(k) - A(i, :)*tmp(:, end));
        des_omega{i} = omega_bar + vc{i};
        gamma{i}(k+1) = gamma{i}(k) + Ts*des_omega{i};
        r{i}(:, k+1) = [rx*cos(gamma{i}(k+1) + gamma0{i}); ry*sin(gamma{i}(k+1) + gamma0{i})];
        des_pos{i}(:, k+1) = r{i}(:, k+1) + l_state_hat{i, k+1}(1:2, :);
    end
    
    % Compute the control action needed for both follower
    for i=1:num_agents
        if k == 1
            upre{i} = zeros(2, 1);
        else
            upre{i} = u{i}(:, k-1);
        end
%         u{i}(:, k) = compute_control_action(eps, f_psi{i}(1, k), f_pos{i, k}, des_pos{i}(:, k+1), des_omega{i}, Kp, gamma{i}(k+1), gamma0{i},rx, ry,...
%                                             l_state_hat{i, k}(3:4, :), act_lim);
        
        % Adding the obstacle avoidance
        x_obs = [];
        y_obs = [];
        r_obs = [];
        for j=1:num_static_obstacles
            x_obs = [x_obs; obstacle_center{j}(1)];
            y_obs = [y_obs; obstacle_center{j}(2)];
            r_obs = [r_obs; obstacle_radius{j}];
        end
        
        % Adding the collision avoidance (among agents)
        for j=1:num_agents
            if j ~= i
                x_obs = [x_obs; f_pos{j, k}(1)];
                y_obs = [y_obs; f_pos{j, k}(2)];
                r_obs = [r_obs; agent_radius];
            end
        end
        
        % Adding the collision avoidance (with target)
        x_obs = [x_obs; qhat{i}(1, k+1)];
        y_obs = [y_obs; qhat{i}(2, k+1)];
        r_obs = [r_obs; agent_radius];

        out = cbf_controller([f_pos{i, k}; f_psi{i}(1, k)], des_pos{i}(:, k+1), des_omega{i}, upre{i}, length(x_obs), x_obs,...
                                    y_obs, r_obs, [min_lin_vel; min_ang_vel], [max_lin_vel; max_ang_vel], eps, Kp, gamma{i}(k+1), ...
                                    gamma0{i},rx, ry, l_state_hat{i, k}(3:4, :), l, agent_radius, num_static_obstacles);
        u{i}(:, k) = out(1:2);

    end
    
    % Apply the control actions to the followers
    tmp_f_pos = {};
    for i=1:num_agents
        [f_pos{i, k+1}, f_rot] = follower_dynamics(f_pos{i, k}, f_psi{i}(1, k), u{i}(:, k), Ts, k);
        f_psi{i}(1, k+1) = atan2(f_rot(2,1), f_rot(1,1));
        tmp_f_pos{end + 1} = f_pos{i, k+1};
    end

    % Update the counter
    k = k+1;
end


%% Plots

% Followers trajectories
f = figure('Units','inches', 'Position', [1,1,2.6,1.6]);
hold on;
box on;
axis equal;
tmp = cell2mat(f_pos);
for i=1:num_agents
    f_position(2*i-1:2*i, :) = tmp(2*i-1:2*i, :);
end
plot_colors = hsv(num_agents + 1);
for i=1:num_agents
    plot(des_pos{i}(1, :), des_pos{i}(2, :), 'Color', plot_colors(i, :), 'LineStyle', ':', 'LineWidth', 0.5);
    plot(f_position(2*i-1, :), f_position(2*i, :), 'Color', plot_colors(i, :), 'LineStyle', '-', 'LineWidth', 0.5);
end

% Leader trajectory
plot(l_state(1, :), l_state(2, :), 'Color', plot_colors(end, :), 'LineStyle', '-', 'LineWidth', 0.5);
plot(qhat{1}(1, :), qhat{1}(2, :), 'Color', plot_colors(end, :), 'LineStyle', ':', 'LineWidth', 0.5);

% % Confidence ellipsoids
% for i=1:num_agents
%     plot_confidence_ellipsoid(l_state_hat{i, 1}(1:2, :), P{i, 1}(1:2, 1:2), 0.99, 1, '', 0.1);    % the initial confidence reigons
% end
% ellips_ids = round(linspace(2, k, 30));
% for i=1:length(ellips_ids)
%     for j=1:num_agents
%         plot_confidence_ellipsoid(l_state_hat{j, ellips_ids(i)}(1:2, :), P{j, ellips_ids(i)}(1:2, 1:2), 0.99, 2, '', 0.1); 
%     end
% end

% Scatter robots
for i=1:num_agents
    scatter(f_position(2*i-1, 1), f_position(2*i, 1), 50, 'filled', 'o', 'MarkerEdgeColor', plot_colors(i, :), 'MarkerFaceColor', plot_colors(i, :));
end
scatter(l_state(1, 1), l_state(2, 1), 50, 'filled', 'square', 'MarkerEdgeColor', plot_colors(end, :), 'MarkerFaceColor', plot_colors(end, :));

% f = get(gca, 'Children');
% f = flip(f);
% names_legend = {};
% names_legend{1} = 'Desired trajectory: Agent 1';
% names_legend{2} = 'Trajectory: Agent 1';
% names_legend{3} = 'Desired trajectory: Agent 2';
% names_legend{4} = 'Trajectory: Agent 2';
% names_legend{5} = ['Desired trajectory: Agent', ' ', num2str(num_agents)];
% names_legend{6} = ['Trajectory: Agent', ' ', num2str(num_agents)];
% names_legend{7} = 'True trajectory: Target';
% names_legend{8} = 'Estimated trajectory: Target';
% names_legend{9} = 'Obstacles';
% legend([f(1), f(2), f(3), f(4), f(2*num_agents-1), f(2*num_agents), f(2*num_agents+1), f(2*num_agents+2), f(end)],...
%     names_legend, 'interpreter', 'latex', 'FontSize', 13);

% Obstacles
% for i=1:num_static_obstacles
%     obs(i) = nsidedpoly(1000, 'Center', obstacle_center{i}', 'Radius', obstacle_radius{i});
%     plot(obs(i), 'FaceColor', 'k', 'FaceAlpha', 0.6);
% end

for i=1:num_static_obstacles
    scatter(obstacle_center{i}(1), obstacle_center{i}(2), 15*obstacle_radius{i}, 'filled', 'o', 'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'k', ...
        'MarkerEdgeAlpha', 0.55, 'MarkerFaceAlpha', 0.55);
end

exportgraphics(f, '3agent_trajectory_cbf.pdf', 'ContentType','vector');


% Control actions 
% figure; 
% hold on;
% grid on;
% box on;
% plot(0:Ts:tf-Ts, u{2}(1, :), 'r', 'LineWidth', 2);
% set(gca,'FontSize',12, 'TickLabelInterpreter', 'latex');
% plot(0:Ts:tf-Ts, u{2}(2, :), 'b', 'LineWidth', 2);


% Estimation error
f = figure('Units','inches', 'Position', [1,1,2.6,1.6]);
for j=1:num_agents
    for i=1:length(qhat{1})
        pos_err(i, j) = norm(qhat{j}(:, i) - l_state(1:2, i));
    end
end
hold on;
grid on;
box on;
for i=1:num_agents
    plot(0:Ts:tf, pos_err(:, i), 'Color', plot_colors(i, :));
end
xlim([0 500])
xlabel('Time (s)');
ylabel('$||p_t - \hat{p}_t|| \,\, (m)$');
exportgraphics(f, '3agent_target_est_err_cbf.pdf', 'ContentType','vector');

% Make animation
if show_animation == 1
    make_animation(P, f_position, l_state, l_state_hat, des_pos, k, f_psi, num_agents, agent_radius, obstacle_center, obstacle_radius);
end