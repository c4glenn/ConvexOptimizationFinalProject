function flower_3d_track_with_optimization()
    % Parameters
    R_outer = 20; % Outer radius
    R_inner = 15; % Inner radius
    W = 30; % Width of straight sections
    H = 10; % Height for elevation in 3D
    nPointsCircle = 150; % Points for semicircles
    nPointsLine = 150; % Points for straight sections

    % Generate outer and inner circular arcs
    thetaLeft = linspace(pi/2, -pi/2, nPointsCircle)'; % Semi-circle (left)
    thetaRight = linspace(-pi/2, pi/2, nPointsCircle)'; % Semi-circle (right)

    % Outer boundary
    leftOuterArc = [-R_outer * cos(thetaLeft) - W/2, R_outer * sin(thetaLeft), linspace(0, H, nPointsCircle)'];
    rightOuterArc = [R_outer * cos(thetaRight) + W/2, R_outer * sin(thetaRight), linspace(0, H, nPointsCircle)'];

    % Inner boundary
    leftInnerArc = [-R_inner * cos(thetaLeft) - W/2, R_inner * sin(thetaLeft), linspace(H, 0, nPointsCircle)'];
    rightInnerArc = [R_inner * cos(thetaRight) + W/2, R_inner * sin(thetaRight), linspace(H, 0, nPointsCircle)'];

    % Generate straight sections
    leftStraightOuter = [linspace(-R_outer - W/2, R_outer + W/2, nPointsLine)', ...
                         -R_outer * ones(nPointsLine, 1), ...
                         linspace(0, 0, nPointsLine)'];
    rightStraightOuter = [linspace(R_outer + W/2, -R_outer - W/2, nPointsLine)', ...
                          R_outer * ones(nPointsLine, 1), ...
                          linspace(H, H, nPointsLine)'];

    leftStraightInner = [linspace(-R_inner - W/2, R_inner + W/2, nPointsLine)', ...
                         -R_inner * ones(nPointsLine, 1), ...
                         linspace(0, 0, nPointsLine)'];
    rightStraightInner = [linspace(R_inner + W/2, -R_inner - W/2, nPointsLine)', ...
                          R_inner * ones(nPointsLine, 1), ...
                          linspace(H, H, nPointsLine)'];

    % Combine the boundaries
    leftBoundary = [leftOuterArc; leftStraightOuter; flipud(leftInnerArc); flipud(leftStraightInner)];
    rightBoundary = [rightOuterArc; rightStraightOuter; flipud(rightInnerArc); flipud(rightStraightInner)];

    % Initial control points (midpoints between left and right boundaries)
    numControlPoints = 9; % Number of Bezier control points
    initialControlPoints = zeros(numControlPoints, 3);
    for i = 1:numControlPoints
        idx = round((i - 1) * (size(leftBoundary, 1) - 1) / (numControlPoints - 1)) + 1;
        idx = min(idx, min(size(leftBoundary, 1), size(rightBoundary, 1))); % Ensure idx doesn't exceed bounds
        initialControlPoints(i, :) = (leftBoundary(idx, :) + rightBoundary(idx, :)) / 2;
    end

    % Optimize the control points
    options = optimoptions('fmincon', 'Display', 'iter', 'Algorithm', 'sqp', ...
                           'MaxFunctionEvaluations', 5000, 'StepTolerance', 1e-9);
    optimizedControlPoints = fmincon(@(x) cost_function(reshape(x, [], 3), leftBoundary, rightBoundary), ...
                                     initialControlPoints(:), [], [], [], [], [], [], [], options);

    % Reshape optimized points
    optimizedControlPoints = reshape(optimizedControlPoints, [], 3);

    % Generate the optimized race line
    raceLine = bezier_curve_3d(optimizedControlPoints, 200); % Smooth Bezier curve

    % Plot the results
    figure;
    hold on;
    plot3(leftBoundary(:,1), leftBoundary(:,2), leftBoundary(:,3), 'r', 'LineWidth', 2, 'DisplayName', 'Left Boundary');
    plot3(rightBoundary(:,1), rightBoundary(:,2), rightBoundary(:,3), 'b', 'LineWidth', 2, 'DisplayName', 'Right Boundary');
    plot3(raceLine(:,1), raceLine(:,2), raceLine(:,3), 'g', 'LineWidth', 2, 'DisplayName', 'Optimal Race Line');
    scatter3(optimizedControlPoints(:,1), optimizedControlPoints(:,2), optimizedControlPoints(:,3), 50, 'k', 'filled', 'DisplayName', 'Control Points');
    legend('Location', 'southoutside', 'Orientation', 'horizontal');
    title('3D Flower Shape Racing Track with Bezier Curve Optimization');
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    grid on;
    axis equal;
    hold off;
end

% Function to generate Bezier curve in 3D
function curve = bezier_curve_3d(controlPoints, nPoints)
    n = size(controlPoints, 1) - 1;
    t = linspace(0, 1, nPoints)';
    curve = zeros(nPoints, 3);
    for i = 0:n
        bernstein = nchoosek(n, i) .* (1 - t).^(n - i) .* t.^i;
        curve = curve + bernstein * controlPoints(i + 1, :);
    end
end

% Cost function for optimization
function cost = cost_function(controlPoints, leftBoundary, rightBoundary)
    curve = bezier_curve_3d(controlPoints, 200);

    % Calculate distances to boundaries
    distToLeft = min(pdist2(curve, leftBoundary), [], 2);
    distToRight = min(pdist2(curve, rightBoundary), [], 2);

    % Calculate the centerline
    centerLine = (leftBoundary + rightBoundary) / 2;
    centerLineInterp = interp1(1:size(centerLine, 1), centerLine, linspace(1, size(centerLine, 1), size(curve, 1)));

    % Penalize deviation from the centerline
    distToCenter = sum(vecnorm(curve - centerLineInterp, 2, 2));

    % Calculate curvature
    dCurve = gradient(curve);
    ddCurve = gradient(dCurve);
    curvature = sum(vecnorm(ddCurve, 2, 2));

    % Combine into a scalar cost
    cost = 0.4 * curvature + 0.6 * distToCenter;
end
