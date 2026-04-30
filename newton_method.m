function [loss_history, theta_history] = newton_method(R, theta0, alpha, target)
    theta = theta0(:);
    lossFunction = @(t) R.Compute(t) - target;
    lossVector = lossFunction(theta);
    loss = norm(lossVector);
    maxIter = 50;
    iter = 0;
    loss_history = zeros(1, maxIter);
    theta_history = zeros(maxIter, numel(theta0));
    loss_history(iter) = loss;
    theta_history(iter, :) = theta.';
    
    theta_wrapped = atan2(sin(theta), cos(theta));
    disp(['Iteration: 0', ', Theta: ', mat2str(round(theta_wrapped.', 2))]);
    pause(3);
    while loss > 0.01 && iter < maxIter
        J = analytical_jacobian(R, theta);
        
        % checking presence of singularity points with SVD (with no
        % countermeasures...)
        [~, S, ~] = svd(J); 
        min_singular_value = min(diag(S));
        if min_singular_value < 1e-6
            disp('Singularity detected!');
        end

        J_inv = pinv(J);
        delta = J_inv * lossVector;
        theta = theta - alpha * delta;

        R.Compute(theta);
        R.Update();
        drawnow;
        pause(0.1);
        
        iter = iter + 1;
        lossVector = lossFunction(theta);
        loss = norm(lossFunction(theta));
        loss_history(iter) = loss;
        theta_history(iter, :) = theta.';
        theta_wrapped = atan2(sin(theta), cos(theta));
        disp(['Iteration: ', num2str(iter-1), ', Loss: ', num2str(loss), ', Theta: ', mat2str(round(theta_wrapped.', 2))]);
    end
    loss_history = loss_history(1:iter);
    theta_history = theta_history(1:iter, :);
    theta_wrapped = atan2(sin(theta), cos(theta));
    disp(['Joint variables of the final position: ', num2str(theta_wrapped.')]);
end
