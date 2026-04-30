classdef Robot < handle
    properties(Access = private)
        Lengths, Points, Members, Joints, Pins
    end
    properties
        Update, Compute
    end

    methods (Access = private)
        function pt = compute(R, theta)
            theta = theta(:);
            t = cumsum(theta);
            n = length(theta);
        
            % compute direction and position vectors for each link
            directions = [cos(t.'); sin(t.')];
            R.Points = [0; 0];  % base joint
            for i = 1:n
                R.Points(:, end+1) = R.Points(:, end) + R.Lengths(i) * directions(:, i); % links
            end

            phi = sum(theta);

            pt = [R.Points(:, end); phi]; % position and orientation
        end

        function update(R)
            R.Members.XData = R.Points(1,:);
            R.Members.YData = R.Points(2,:);
            R.Joints.XData = R.Points(1,:);
            R.Joints.YData = R.Points(2,:);
            R.Pins.XData = R.Points(1,1:end-1);
            R.Pins.YData = R.Points(2,1:end-1);
        end
    end

    methods
        function R = Robot(theta)
            R.Lengths = [1, 1, 1];
            plot([-1, 1], [0,0], Color=0.6*[1,1,1], LineWidth=15); % base

            compute(R, theta);
            R.Members = plot(R.Points(1,:), R.Points(2,:), '-k', LineWidth=5);
            R.Joints = plot(R.Points(1,:), R.Points(2,:), 'ok', ...
                LineWidth=2, MarkerFaceColor='w', MarkerSize=15);
            R.Pins = plot(R.Points(1,1:end-1), R.Points(2,1:end-1), ...
                'ok', LineWidth=5, MarkerFaceColor='k', MarkerSize=4);
            R.Update = @()update(R);
            R.Compute = @(theta)compute(R, theta);
        end

        function isReachable = isInWorkspace(R, point)
            % distance from base to target
            d = norm(point);
            
            % Compute workspace bounds
            max_reach = sum(R.Lengths);
            sorted_lengths = sort(R.Lengths, 'descend');
            longest = sorted_lengths(1);
            sum_others = sum(sorted_lengths(2:end));
            
            % Minimum possible reach (handle cases where the longest link dominates)
            if longest > sum_others
                min_reach = longest - sum_others;
            else
                min_reach = 0;
            end
            
            isReachable = (d >= min_reach) && (d <= max_reach);
        end
        
        function J = analytical_jacobian(R, theta)
            l1 = R.Lengths(1); 
            l2 = R.Lengths(2); 
            l3 = R.Lengths(3);
            
            q1 = theta(1); 
            q2 = theta(2); 
            q3 = theta(3);
            
            q12 = q1 + q2;
            q123 = q1 + q2 + q3;
            
            % partial derivatives of the Jacobian
            dx_dq1 = -l1*sin(q1) - l2*sin(q12) - l3*sin(q123);
            dx_dq2 = -l2*sin(q12) - l3*sin(q123);
            dx_dq3 = -l3*sin(q123);
            
            dy_dq1 = l1*cos(q1) + l2*cos(q12) + l3*cos(q123);
            dy_dq2 = l2*cos(q12) + l3*cos(q123);
            dy_dq3 = l3*cos(q123);
            
            J = [dx_dq1, dx_dq2, dx_dq3;
                 dy_dq1, dy_dq2, dy_dq3,
                 1       1,      1];
        end

    end
end
