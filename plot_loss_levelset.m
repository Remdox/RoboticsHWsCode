function plot_loss_levelset(R, target, theta, learningRate, methodName)
    [T1_deg, T2_deg] = meshgrid(0:5:360, 0:5:360); 
    loss = zeros(size(T1_deg));
    
    for i = 1:numel(T1_deg)
        theta1 = deg2rad(T1_deg(i));
        theta2 = deg2rad(T2_deg(i));
        theta3 = -(theta1 + theta2); % Constraint phi = 0
        pos = R.Compute([theta1, theta2, theta3]);
        loss(i) = norm(pos - target);
    end

    figLevelSet = figure('Color', 'w');
    contour(T1_deg, T2_deg, loss, 20, 'LineWidth', 1.5);
    colorbar;
    hold on;
    
    theta_deg = zeros(size(theta, 1), 2);
    for i = 1:size(theta, 1)
        theta1 = theta(i, 1);
        theta2 = theta(i, 2);
        theta3 = -(theta1 + theta2); % Keep constraint
        theta_deg(i, :) = rad2deg([theta1, theta2]);
    end
    
    plot(theta_deg(:,1), theta_deg(:,2), 'k.-', 'LineWidth', 2, 'MarkerSize', 15, 'DisplayName', 'Traiettoria');
    scatter(theta_deg(1,1), theta_deg(1,2), 100, 'bo', 'filled', 'DisplayName', 'Inizio');
    scatter(theta_deg(end,1), theta_deg(end,2), 100, 'r^', 'filled', 'DisplayName', 'Fine');
    
    title(sprintf('Level-Set - Target [%d, %d, %.1f] - %s', target(1), target(2), target(3), methodName));
    xlabel('\theta_1 [°]');
    ylabel('\theta_2 [°]');
    axis equal;
    xlim([0 360]);
    ylim([0 360]);
    grid on;
    legend('Location', 'best');
    
    if ~exist('plots', 'dir')
        mkdir('plots');
    end
    saveas(figLevelSet, fullfile('plots', sprintf('loss_LevelSet_%.1f.png', learningRate)));
end