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
    
    if ~exist('loss', 'dir')
        mkdir('loss');
    end
    % Replace the old string conversion with this:
    theta0_str = mat2str(theta0); 
    % Remove brackets and replace semicolons/spaces with underscores
    theta0_str = regexprep(theta0_str, '[\[\] ]', ''); 
    theta0_str = strrep(theta0_str, ';', '_');
    
    % Now saveas will receive a clean path like 'plots_2026/loss_GRAD_0_0_0_0.15.png'
    saveas(figLoss, fullfile('loss', ['loss_', methodName, '_', theta0_str, '_', num2str(learningRate), '.png']));
end