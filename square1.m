function bezier_optimization_rectangular_round()
    % Define rectangular track with rounded corners
    [leftBoundary, rightBoundary] = rectangular_round_track();

    % Initial control points (midpoints between left and right boundaries)
    numControlPoints = 20; % Number of Bezier control points
    initialControlPoints = zeros(numControlPoints, 2);
    boundarySize = size(leftBoundary, 1); % Size of the boundary array

    for i = 1:numControlPoints
        % Calculate index ensuring it stays within boundary limits
        idx = round((i - 1) * (boundarySize - 1) / (numControlPoints - 1)) + 1;
        idx = min(max(idx, 1), boundarySize); % Ensure idx stays within bounds
        initialControlPoints(i, :) = (leftBoundary(idx, :) + rightBoundary(idx, :)) / 2;
    end

    % Optimize the control points
    options = optimoptions('fmincon', 'Display', 'iter', 'Algorithm', 'sqp', ...
                           'MaxFunctionEvaluations', 5000, 'StepTolerance', 1e-9);
    optimizedControlPoints = fmincon(@(x) cost_function(reshape(x, [], 2), leftBoundary, rightBoundary), ...
                                     initialControlPoints(:), [], [], [], [], [], [], [], options);

    % Reshape optimized points
    optimizedControlPoints = reshape(optimizedControlPoints, [], 2);

    % Generate the optimized race line
    raceLine = bezier_curve(optimizedControlPoints, 200); % Smooth Bezier curve

    % Plot the results
    figure;
    hold on;
    plot(leftBoundary(:,1), leftBoundary(:,2), 'r', 'LineWidth', 2, 'DisplayName', 'Left Boundary');
    plot(rightBoundary(:,1), rightBoundary(:,2), 'b', 'LineWidth', 2, 'DisplayName', 'Right Boundary');
    plot(raceLine(:,1), raceLine(:,2), 'g', 'LineWidth', 2, 'DisplayName', 'Optimal Race Line');
    scatter(optimizedControlPoints(:,1), optimizedControlPoints(:,2), 50, 'k', 'filled', 'DisplayName', 'Control Points');
    legend('Location', 'southoutside', 'Orientation', 'horizontal');
    title('Optimal Race Line on Rectangular Round Track using Bezier Curve');
    xlabel('X');
    ylabel('Y');
    axis equal;
    xlim([-30, 30]); % Adjust axes limits for better visualization
    ylim([-20, 40]);
    hold off;
end

% Function to define rectangular track with rounded corners
function [leftBoundary, rightBoundary] = rectangular_round_track()
    % Parameters for track
    lengthStraight = 40; % Length of straight sections
    widthStraight = 20; % Width of straight sections
    cornerRadius = 5; % Radius of rounded corners
    nPointsCorner = 50; % Points per corner
    nPointsStraight = 100; % Points per straight section

    % Generate corners
    corners = [lengthStraight/2, widthStraight/2;
               -lengthStraight/2, widthStraight/2;
               -lengthStraight/2, -widthStraight/2;
                lengthStraight/2, -widthStraight/2];

    leftBoundary = [];
    rightBoundary = [];

    for i = 1:4
        % Current corner center
        cornerCenter = corners(i, :);

        % Angle range for rounded corner
        thetaStart = (i-1)*pi/2;
        thetaEnd = i*pi/2;
        theta = linspace(thetaStart, thetaEnd, nPointsCorner);

        % Generate corner points
        outerCorner = cornerCenter + cornerRadius * [cos(theta)', sin(theta)'];
        innerCorner = cornerCenter + (cornerRadius - 2) * [cos(theta)', sin(theta)'];

        % Generate straight sections
        nextCornerIdx = mod(i, 4) + 1; % Next corner index
        straightOuterStart = outerCorner(end, :);
        straightOuterEnd = corners(nextCornerIdx, :) + cornerRadius * [cos(thetaEnd), sin(thetaEnd)];
        straightOuter = [linspace(straightOuterStart(1), straightOuterEnd(1), nPointsStraight)', ...
                         linspace(straightOuterStart(2), straightOuterEnd(2), nPointsStraight)'];

        straightInnerStart = innerCorner(end, :);
        straightInnerEnd = corners(nextCornerIdx, :) + (cornerRadius - 2) * [cos(thetaEnd), sin(thetaEnd)];
        straightInner = [linspace(straightInnerStart(1), straightInnerEnd(1), nPointsStraight)', ...
                         linspace(straightInnerStart(2), straightInnerEnd(2), nPointsStraight)'];

        % Append to boundaries
        leftBoundary = [leftBoundary; outerCorner; straightOuter];
        rightBoundary = [rightBoundary; innerCorner; straightInner];
    end
end

% Function to generate Bezier curve
function curve = bezier_curve(controlPoints, nPoints)
    n = size(controlPoints, 1) - 1;
    t = linspace(0, 1, nPoints)';
    curve = zeros(nPoints, 2);
    for i = 0:n
        bernstein = nchoosek(n, i) .* (1 - t).^(n - i) .* t.^i; % Bernstein basis
        curve = curve + bernstein * controlPoints(i + 1, :);
    end
end

% Cost function for optimization
function cost = cost_function(controlPoints, leftBoundary, rightBoundary)
    curve = bezier_curve(controlPoints, 200); % Evaluate curve

    % Calculate distances to boundaries
    distToLeft = min(pdist2(curve, leftBoundary), [], 2); % Minimum distance to left boundary
    distToRight = min(pdist2(curve, rightBoundary), [], 2); % Minimum distance to right boundary

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
    cost = 0.4 * curvature + 0.6 * distToCenter; % Adjust weights as needed
end
