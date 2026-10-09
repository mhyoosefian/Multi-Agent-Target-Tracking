function make_animation(P, f_position, l_state, l_state_hat, des_pos, k, f_psi, num_agents, agent_radius, obstacle_center, obstacle_radius)

l = agent_radius/sqrt(2);    % quadrotor arm length
l_target = 2;
plot_colors = hsv(num_agents + 1);
num_obstacles = length(obstacle_radius);

max_ylim = 0;
for i=1:num_agents
    max_ylim = max(f_position(2*i, 1), max_ylim);
end

figure;
box on;
myVideo = VideoWriter('myVideoFile.avi');
myVideo.FrameRate = 15;
myVideo.Quality = 85;
open(myVideo)
for i=1:k

    % Trackers arm tip positions
    for j=1:num_agents
        tracker_center{j} = f_position(2*j-1:2*j, i);
        tracker_horizontal_x{j} = [tracker_center{j}(1) + l*cos(f_psi{j}(i)); tracker_center{j}(1) - l*cos(f_psi{j}(i))];
        tracker_horizontal_y{j} = [tracker_center{j}(2) + l*sin(f_psi{j}(i)); tracker_center{j}(2) - l*sin(f_psi{j}(i))];
        tracker_vertical_x{j} = [tracker_center{j}(1) + l*sin(f_psi{j}(i)); tracker_center{j}(1) - l*sin(f_psi{j}(i))];
        tracker_vertical_y{j} = [tracker_center{j}(2) - l*cos(f_psi{j}(i)); tracker_center{j}(2) + l*cos(f_psi{j}(i))];
    end

    % Target arm tip positions
    target_center = l_state(1:2, i);
    target_horizontal_x = [target_center(1) + l_target; target_center(1) - l_target];
    target_horizontal_y = [target_center(2); target_center(2)];
    target_vertical_x = [target_center(1); target_center(1)];
    target_vertical_y = [target_center(2) - l_target; target_center(2) + l_target];

    % Plot tracker 1
    plot(tracker_horizontal_x{1}, tracker_horizontal_y{1}, 'Color', plot_colors(1, :), 'LineWidth', 2);
    hold on;
    grid on;
    set(gca,'FontSize',12, 'TickLabelInterpreter', 'latex');
    plot(tracker_vertical_x{1}, tracker_vertical_y{1}, 'Color', plot_colors(1, :), 'LineWidth', 2);
    
    % Plot other trackers
    for j=2:num_agents
        plot(tracker_horizontal_x{j}, tracker_horizontal_y{j}, 'Color', plot_colors(j, :), 'LineWidth', 2);
        plot(tracker_vertical_x{j}, tracker_vertical_y{j}, 'Color', plot_colors(j, :), 'LineWidth', 2);
    end

    % Plot target
    plot(target_horizontal_x, target_horizontal_y, 'Color', plot_colors(end, :), 'LineWidth', 3);
    plot(target_vertical_x, target_vertical_y, 'Color', plot_colors(end, :), 'LineWidth', 3);

    % Plot confidence ellipsoids
    for j=1:num_agents
        qhat = l_state_hat{j, i}(1:2, :);
        plot_confidence_ellipsoid(qhat, P{j, i}(1:2, 1:2), 0.99, 2, '', 0.1); 
    end
    
    % Plot the true and estimated leader trajectory
    plot(l_state(1, 1:i), l_state(2, 1:i), 'Color', plot_colors(end, :), 'LineWidth', 2);
    tmp = cell2mat(l_state_hat);
    for j=1:num_agents
        plot(tmp(4*j-3, 1:i), tmp(4*j-2, 1:i), 'Color', plot_colors(end, :), 'LineWidth', 0.5);
    end
    
    % Plot desired trajectores of trackers
    for j=1:num_agents
        plot(des_pos{j}(1, 1:i), des_pos{j}(2, 1:i), 'Color', plot_colors(j, :), 'LineWidth', 1, 'LineStyle', '--');
    end

    % Plot obstacles
%     for j=1:num_obstacles
%         obs(j) = nsidedpoly(1000, 'Center', obstacle_center{j}', 'Radius', obstacle_radius{j});
%         plot(obs(j), 'FaceColor', 'k', 'FaceAlpha', 0.7);
%     end
    
    % Some settings
    for j=1:num_agents
        if f_position(2*j, i) >= max_ylim
            max_ylim = f_position(2*j, i);
        end
    end
    axis([-50 50 -40 max_ylim+1.5*l]);
    hold off
    pause(0.01)
    frame = getframe(gcf);
    writeVideo(myVideo, frame);
end
close(myVideo)