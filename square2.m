function square2_fixed_track()
    % Define round-corner rectangular track boundaries
    [leftBoundary, rightBoundary] = round_rectangular_track();

    % Adjust rightBoundary to fit inside leftBoundary
    scaleFactor = 0.9; % Scale factor to reduce the size of the blue rectangle
    rightBoundary = (rightBoundary - mean(leftBoundary)) * scaleFactor + mean(leftBoundary);

    % Initialize control points (aligned with the track centerline)
    numControlPoints = 50; % Increase number for better resolution
    initialControlPoints = zeros(numControlPoints, 2);
    boundarySize = size(leftBoundary, 1);

    for i = 1:numControlPoints
        idx = round((i - 1) * (boundarySize - 1) / (numControlPoints - 1)) + 1;
        idx = min(max(idx, 1), boundarySize); % Ensure idx stays within bounds
        initialControlPoints(i, :) = (leftBoundary(idx, :) + rightBoundary(idx, :)) / 2;
    end

    % Optimize the control points
    options = optimoptions('fmincon', 'Display', 'iter', 'Algorithm', 'sqp', ...
                           'MaxFunctionEvaluations', 20000, 'StepTolerance', 1e-9);
    optimizedControlPoints = fmincon(@(x) cost_function(reshape(x, [], 2), leftBoundary, rightBoundary), ...
                                     initialControlPoints(:), [], [], [], [], [], [], [], options);

    % Reshape optimized points
    optimizedControlPoints = reshape(optimizedControlPoints, [], 2);

    % Generate the optimized race line
    raceLine = bezier_curve(optimizedControlPoints, 500);

    % Plot the results
    figure;
    hold on;
    plot(leftBoundary(:,1), leftBoundary(:,2), 'r', 'LineWidth', 2, 'DisplayName', 'Left Boundary');
    plot(rightBoundary(:,1), rightBoundary(:,2), 'b', 'LineWidth', 2, 'DisplayName', 'Right Boundary');
    plot(raceLine(:,1), raceLine(:,2), 'g', 'LineWidth', 2, 'DisplayName', 'Optimal Race Line');
    scatter(optimizedControlPoints(:,1), optimizedControlPoints(:,2), 50, 'k', 'filled', 'DisplayName', 'Control Points');
    legend('Location', 'southoutside', 'Orientation', 'horizontal');
    title('Improved Track with Inner Boundary Fitting Properly');
    xlabel('X');
    ylabel('Y');
    axis equal;
    xlim([-35, 35]);
    ylim([-25, 25]);
    hold off;
end

% Function to generate round-corner rectangular track boundaries
function [leftBoundary, rightBoundary] = round_rectangular_track()
    % Parameters for the track
    longSide = 30; % Length of the long sides
    shortSide = 15; % Length of the short sides
    cornerRadius = 5; % Radius of the rounded corners
    nPointsCorner = 50; % Number of points for each rounded corner
    nPointsStraight = 100; % Number of points for each straight section

    % Define corner centers
    corners = [
        longSide / 2 - cornerRadius, shortSide / 2 - cornerRadius;  % Top-right
        -longSide / 2 + cornerRadius, shortSide / 2 - cornerRadius; % Top-left
        -longSide / 2 + cornerRadius, -shortSide / 2 + cornerRadius; % Bottom-left
        longSide / 2 - cornerRadius, -shortSide / 2 + cornerRadius  % Bottom-right
    ];

    % Initialize boundaries
    leftBoundary = [];
    rightBoundary = [];

    % Generate rounded corners and straight sections
    for i = 1:4
        % Define start and end angles for the corner
        thetaStart = (i - 1) * pi / 2;
        thetaEnd = i * pi / 2;
        theta = linspace(thetaStart, thetaEnd, nPointsCorner);

        % Generate points for the corner
        cornerPoints = corners(i, :) + cornerRadius * [cos(theta)', sin(theta)'];

        % Define the straight section
        if i < 4
            nextCorner = corners(i + 1, :);
        else
            nextCorner = corners(1, :); % Wrap around to the first corner
        end
        if mod(i, 2) == 1
            % Horizontal straight sections
            straightPoints = [linspace(cornerPoints(end, 1), nextCorner(1), nPointsStraight)', ...
                              repmat(cornerPoints(end, 2), nPointsStraight, 1)];
        else
            % Vertical straight sections
            straightPoints = [repmat(cornerPoints(end, 1), nPointsStraight, 1), ...
                              linspace(cornerPoints(end, 2), nextCorner(2), nPointsStraight)'];
        end

        % Append points to the boundaries
        leftBoundary = [leftBoundary; cornerPoints; straightPoints];
        rightBoundary = [rightBoundary; cornerPoints; straightPoints]; % Placeholder for now
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

% Updated cost function
function cost = cost_function(controlPoints, leftBoundary, rightBoundary)
    curve = bezier_curve(controlPoints, 500); % Evaluate curve

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
    cost = 0.7 * distToCenter + 0.3 * curvature; % Heavily penalize centerline deviation
end
