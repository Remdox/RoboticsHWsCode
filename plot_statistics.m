function plot_statistics(theta0, lossHistory, thetaHistory, methodName, learningRate)
    figLoss = figure('Color', 'w');
    
    % loss plot
    subplot(2,1,1);
    plot(1:length(lossHistory), lossHistory, 'b-', 'LineWidth', 1.5);
    title(['Loss - ', methodName, '   LR: ', num2str(learningRate)]);
    ylabel('Cartesian position error');
    xlabel('Iterations');
    yline(0.1, '--', 'Threshold 0.1', 'Color', 'k', 'LineWidth', 1.5);
    hold off;
    grid on;
    
    % joint variables plot
    subplot(2,1,2);
    hold on;
    plot(1:size(thetaHistory, 1), thetaHistory(:,1), 'r-', 'LineWidth', 1.5);
    plot(1:size(thetaHistory, 1), thetaHistory(:,2), 'g--', 'LineWidth', 1.5);
    plot(1:size(thetaHistory, 1), thetaHistory(:,3), 'b-.', 'LineWidth', 1.5);
    title('Evolution of the joint variables');
    xlabel('Iterations');
    ylabel('q [rad]');
    legend('q_{1}', 'q_{2}', 'q_{3}');
    grid on;
    
    if ~exist('plots', 'dir')
        mkdir('plots');
    end
    theta0_str = strrep(mat2str(theta0), ' ', '_');
    saveas(figLoss, fullfile('plots', ['loss_', methodName, '_', theta0_str, '_', num2str(learningRate), '.png']));
end