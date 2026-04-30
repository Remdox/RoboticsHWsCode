fig = figure(Color = 'w', Position=[300, 250,560,550]);
axis equal off; 
axis([-3, 7, -0.5, 9.5]); 
hold on;
tplt = plot(5,5,'or',MarkerFaceColor='r');

% parameters: target to reach, joint variables, learning rate (only for gradient method).
target = [1; 2; 1.57];     % target pose to reach (radiants)
theta0_deg = [90; 90; 90]; % initial conditions 0,0,0 | 90,90,90 (degrees)
theta0 = deg2rad(theta0_deg);
learningRate = 0.1 ;

theta = theta0;
robotArm = Robot(theta);

tplt.XData = target(1); 
tplt.YData = target(2);
lrtxt = text(-2, 9, "LearningRate = " + learningRate, FontSize=15, Interpreter="latex");

if robotArm.isInWorkspace(target)
    disp("Point reachable. Proceeding the computation of inverse kinematics...")
    
    disp('@@@ Gradient Method @@@');

    isConvergentG = false;
    [lossHistoryG, thetaHistoryG, isConvergentG] = gradient_method(robotArm, theta, target, learningRate);
    
    plot_statistics(theta0_deg, lossHistoryG, thetaHistoryG, 'GRAD', learningRate);
    theta0_str = strrep(mat2str(theta0_deg), ' ', '_');
    saveas(fig, fullfile('plots', ['3R_Robot_gradient_', theta0_str, '_', num2str(learningRate), '.png']));
    plot_loss_levelset(robotArm, target, thetaHistoryG, learningRate, "GRAD");
    pause(2);

    theta = theta0;
    disp('@@@ Newton Method @@@');
    robotArm.Compute(theta)
    robotArm.Update();
    learningRate = 1;
    [lossHistoryN, thetaHistoryN] = newton_method(robotArm, theta, learningRate, target);
    
    plot_statistics(theta0_deg, lossHistoryN, thetaHistoryN, 'NEWTON', learningRate);
    saveas(fig, fullfile('plots', ['3R_Robot_Newton_', theta0_str, '_', num2str(learningRate), '.png']));
    plot_loss_levelset(robotArm, target, thetaHistoryN, learningRate, "NEWTON");
    pause(30);
else
    disp("NOT REACHABLE point. Aborting...")
end
close all;