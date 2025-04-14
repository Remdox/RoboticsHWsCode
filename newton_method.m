function [loss_history, theta_history] = newton_method(R, theta0, target)
    theta = theta0(:);
    lossFunction = @(t) R.Compute(t) - target;
    lossVector = lossFunction(theta);
    loss = norm(lossVector);
    maxIter = 20;
    iter = 0;
    loss_history = zeros(1, maxIter);
    theta_history = zeros(maxIter, numel(theta0));
    alpha = 1;

    disp('Iterazione 0');
    pause(3);
    while loss > 0.01 && iter < maxIter
        J = analytical_jacobian(R, theta);
        
        % checking presence of singularity points with SVD (with no
        % countermeasures...)
        [~, S, ~] = svd(J); 
        min_singular_value = min(diag(S));
        if min_singular_value < 1e-6
            disp('Singolarità rilevata!');
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
        disp(['Iteration: ', num2str(iter), ', Loss: ', num2str(loss)]);
    end
    loss_history = loss_history(1:iter);
    theta_history = theta_history(1:iter, :);
    disp(['Joint variables of the final position: ', num2str(theta.')]);
end
