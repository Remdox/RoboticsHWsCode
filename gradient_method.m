function [loss_history, theta_history, isConvergent] = gradient_method(R, theta0, target, learningRate)
    theta = theta0(:); 
    lossFunction = @(t) R.Compute(t) - target;
    loss = norm(lossFunction(theta)); % Stopping criteria: cartesian error < epsilon = 0.1
    maxIter = 1000;
    iter = 1;
    loss_history = zeros(1, maxIter+1);
    theta_history = zeros(maxIter+1, numel(theta0));
    loss_history(iter) = loss;
    theta_history(iter, :) = theta.';
    isConvergent = false;

    disp(['Iteration: 0', ', Theta: ', mat2str(round(theta.', 2))]);
    pause(3);
    while loss > 0.01 && iter < maxIter
        J = analytical_jacobian(R, theta);
        
        % some debug leftovers...
        % J_numerical = numerical_jacobian(R, theta, 1e-3);
        % error = norm(J - J_numerical, 'fro');
        % disp(['Errore Jacobiano: ', num2str(error)]);
        % disp('Jacobiana analitica: '); disp(J);
        % disp('Jacobiana numerica: '); disp(J_numerical);

        % checking presence of singularity points with SVD (with no
        % countermeasures...)
        [U, S, V] = svd(J);
        min_singular_value = min(diag(S));
        if min_singular_value < 1e-6
            disp('Singularity detected!');
        end

        % update
        lossVector = lossFunction(theta);
        disp(['iteration:', num2str(iter-1), ', loss: ',num2str(loss), ', Theta: ', mat2str(round(theta.', 2))]);
        theta = theta - learningRate * J' * lossVector;
        
        R.Compute(theta);
        R.Update();
        %drawnow;
        pause(0.1);
        
        iter = iter + 1;
        loss = norm(lossFunction(theta)); % Re-computing error after having adjusted theta
        loss_history(iter) = loss;
        theta_history(iter, :) = theta.';
    end
    % Cut histories to actual number of iterations reached on convergence
    loss_history = loss_history(1:iter);
    theta_history = theta_history(1:iter, :);
    theta = theta.';

    if loss <= 0.1
        isConvergent = true;
        disp('Convergence reached.');
        disp(['Number of iterations: ', num2str(iter), ', Loss: ', num2str(loss), ', Theta: ', mat2str(round(theta.', 2))]);
    else
        disp(['!!! Convergence NOT REACHED !!! After ', num2str(iter), ' iterations, with loss ', num2str(loss)])
    end
    disp(['Joint variables of the final position: ', num2str(theta)]);
end

function J = numerical_jacobian(R, theta_rad, epsilon)
    n = numel(theta_rad);
    m = numel(R.Compute(theta_rad));
    J = zeros(m, n);
    
    for i = 1:n
        theta_perturbed = theta_rad;
        theta_perturbed(i) = theta_perturbed(i) + epsilon;
        J(:, i) = (R.Compute(theta_perturbed) - R.Compute(theta_rad)) / epsilon;
    end
end