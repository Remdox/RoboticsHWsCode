function plot_loss_levelset(R, target, theta, learningRate, methodName)
    [T1_rad, T2_rad] = meshgrid(0:0.1:2*pi, 0:0.1:2*pi); 
    loss_map = zeros(size(T1_rad));
    
    for i = 1:numel(T1_rad)
        t1 = T1_rad(i);
        t2 = T2_rad(i);
        t3 = target(3) - (t1 + t2); 
        
        pos = R.Compute([t1, t2, t3]);
        loss_map(i) = norm(pos - target);
    end
    
    figLevelSet = figure('Color', 'w');
    contour(T1_rad, T2_rad, loss_map, 20, 'LineWidth', 1.5, 'DisplayName', 'Loss Surface');
    colorbar;
    hold on;
    
    traj_rad = mod(theta(:, 1:2), 2*pi);
    
    plot(traj_rad(:,1), traj_rad(:,2), 'k.-', 'LineWidth', 1, 'MarkerSize', 8, 'Color', [0.3 0.3 0.3], 'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'c', 'DisplayName', 'Trajectory');
    scatter(traj_rad(1,1), traj_rad(1,2), 100, 'bo', 'filled', 'DisplayName', 'Start');
    scatter(traj_rad(end,1), traj_rad(end,2), 100, 'r^', 'filled', 'DisplayName', 'End');
    
    title(sprintf('Level-Set (rad) - Target [%.1f, %.1f, %.1f] - %s', ...
          target(1), target(2), target(3), methodName));
    xlabel('\theta_1 [rad]');
    ylabel('\theta_2 [rad]');
    
    axis equal;
    xlim([0 2*pi]);
    ylim([0 2*pi]);
    
    xticks([0 pi/2 pi 3*pi/2 2*pi]);
    xticklabels({'0','\pi/2','\pi','3\pi/2','2\pi'});
    yticks([0 pi/2 pi 3*pi/2 2*pi]);
    yticklabels({'0','\pi/2','\pi','3\pi/2','2\pi'});
    
    grid on;
    legend('Location', 'best');
    
    if ~exist('loss', 'dir')
        mkdir('loss');
    end
    saveas(figLevelSet, fullfile('loss', sprintf('LevelSet_%s_LR%.2f.png', methodName, learningRate)));
end