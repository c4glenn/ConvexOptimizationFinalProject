function bezier_optimization_oval_fixed_v3()
    % Define oval track boundaries
    [leftBoundary, rightBoundary] = oval_track();

    % Initial control points (midpoints between left and right boundaries)
    numControlPoints = 25; % Increase to improve flexibility of the Bezier curve
    initialControlPoints = zeros(numControlPoints, 2);
    for i = 1:numControlPoints
        idx = round((i - 1) * (size(leftBoundary, 1) - 1) / (numControlPoints - 1)) + 1;
        initialControlPoints(i, :) = (leftBoundary(idx, :) + rightBoundary(idx, :)) / 2;
    end

    % Debug: Display initial control points
    disp('Initial Control Points:');
    disp(initialControlPoints);

    % Optimize the control points
    options = optimoptions('fmincon', 'Display', 'iter', 'Algorithm', 'sqp', ...
                           'MaxFunctionEvaluations', 5000, 'StepTolerance', 1e-6);
    optimizedControlPoints = fmincon(@(x) cost_function(reshape(x, [], 2), leftBoundary, rightBoundary), ...
                                     initialControlPoints(:), [], [], [], [], [], [], [], options);

    % Reshape optimized points
    optimizedControlPoints = reshape(optimizedControlPoints, [], 2);

    % Debug: Display optimized control points
    disp('Optimized Control Points:');
    disp(optimizedControlPoints);

    % Generate the optimized race line
    raceLine = bezier_curve(optimizedControlPoints, 100);

    % Plot the results
    figure;
    hold on;
    plot(leftBoundary(:,1), leftBoundary(:,2), 'r', 'LineWidth', 2, 'DisplayName', 'Left Boundary');
    plot(rightBoundary(:,1), rightBoundary(:,2), 'b', 'LineWidth', 2, 'DisplayName', 'Right Boundary');
    plot(raceLine(:,1), raceLine(:,2), 'g', 'LineWidth', 2, 'DisplayName', 'Optimal Race Line');
    scatter(optimizedControlPoints(:,1), optimizedControlPoints(:,2), 'k', 'filled', 'DisplayName', 'Control Points');
    legend show;
    title('Optimal Race Line on Oval Track using Bezier Curve');
    xlabel('X');
    ylabel('Y');
    axis equal;
    xlim([-12, 12]); % Adjust axes limits for better visualization
    ylim([-7, 7]);
    hold off;
end

% Function to generate Bezier curve
function curve = bezier_curve(controlPoints, nPoints)
    n = size(controlPoints, 1) - 1; % Degree of the Bezier curve
    t = linspace(0, 1, nPoints)'; % Parameter values
    curve = zeros(nPoints, 2); % Initialize curve points
    for i = 0:n
        bernstein = nchoosek(n, i) .* (1 - t).^(n - i) .* t.^i; % Bernstein basis polynomial
        curve = curve + bernstein * controlPoints(i + 1, :); % Weighted sum of control points
    end
end

% Function to define oval track boundaries
function [leftBoundary, rightBoundary] = oval_track()
    theta = linspace(0, 2*pi, 200)'; % Parameter for the oval shape
    a_outer = 10; b_outer = 5; % Semi-major and semi-minor axes of outer boundary
    a_inner = 9;  b_inner = 4.5; % Semi-major and semi-minor axes of inner boundary

    leftBoundary = [a_outer * cos(theta), b_outer * sin(theta)];
    rightBoundary = [a_inner * cos(theta), b_inner * sin(theta)];
end

% Cost function for optimization
function cost = cost_function(controlPoints, leftBoundary, rightBoundary)
    curve = bezier_curve(controlPoints, 100); % Generate the Bezier curve

    % Calculate distances to boundaries
    distToLeft = min(pdist2(curve, leftBoundary), [], 2);  % Minimum distance to left boundary
    distToRight = min(pdist2(curve, rightBoundary), [], 2); % Minimum distance to right boundary

    % Calculate the centerline
    centerLine = (leftBoundary + rightBoundary) / 2;
    centerLineInterp = interp1(1:size(centerLine, 1), centerLine, linspace(1, size(centerLine, 1), size(curve, 1)));

    % Penalize distances from the centerline
    distToCenter = sum(vecnorm(curve - centerLineInterp, 2, 2)); % Penalize deviation from the center

    % Calculate curvature
    dCurve = gradient(curve);
    ddCurve = gradient(dCurve);
    curvature = sum(vecnorm(ddCurve, 2, 2)); % Aggregate curvature

    % Combine into a scalar cost: minimize curvature, maximize distance
    cost = 0.5 * curvature + 0.5 * distToCenter; % Adjust weights as needed
end
