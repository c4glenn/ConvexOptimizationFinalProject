function soccer_field_track_corrected()
    % Define the track boundaries
    [leftBoundary, rightBoundary] = soccer_field_boundaries_corrected();

    % Interpolate rightBoundary to match leftBoundary size
    if size(leftBoundary, 1) ~= size(rightBoundary, 1)
        fprintf('Mismatch in boundary sizes. Interpolating Right Boundary...\n');
        rightBoundary = interp1(1:size(rightBoundary, 1), rightBoundary, ...
                                linspace(1, size(rightBoundary, 1), size(leftBoundary, 1)));
    end

    % Validate boundaries
    fprintf('Validating Left and Right Boundaries...\n');
    if any(isnan(leftBoundary), 'all') || any(isinf(leftBoundary), 'all')
        error('Left Boundary contains NaN or Inf values.');
    end
    if any(isnan(rightBoundary), 'all') || any(isinf(rightBoundary), 'all')
        error('Right Boundary contains NaN or Inf values.');
    end

    % Initialize control points
    numControlPoints = 25; % Adjust based on track complexity
    boundarySize = size(leftBoundary, 1);

    % Initialize control points on the track centerline
    initialControlPoints = zeros(numControlPoints, 2);
    for i = 1:numControlPoints
        idx = round((i - 1) * (boundarySize - 1) / (numControlPoints - 1)) + 1;
        idx = min(max(idx, 1), boundarySize); % Ensure idx stays within bounds
        initialControlPoints(i, :) = (leftBoundary(idx, :) + rightBoundary(idx, :)) / 2;
    end

    % Debugging: Print initial control points
    fprintf('Initial Control Points:\n');
    disp(initialControlPoints);

    % Optimize the control points
    options = optimoptions('fmincon', 'Display', 'iter', 'Algorithm', 'sqp', ...
                           'MaxFunctionEvaluations', 50000, 'StepTolerance', 1e-10);
    optimizedControlPoints = fmincon(@(x) cost_function(reshape(x, [], 2), leftBoundary, rightBoundary), ...
                                     initialControlPoints(:), [], [], [], [], [], [], [], options);

    % Reshape optimized points
    optimizedControlPoints = reshape(optimizedControlPoints, [], 2);

    % Generate the optimized race line
    raceLine = bezier_curve(optimizedControlPoints, 1000);

    % Plot the results
    figure;
    hold on;
    plot(leftBoundary(:,1), leftBoundary(:,2), 'r', 'LineWidth', 2, 'DisplayName', 'Left Boundary');
    plot(rightBoundary(:,1), rightBoundary(:,2), 'b', 'LineWidth', 2, 'DisplayName', 'Right Boundary');
    plot(raceLine(:,1), raceLine(:,2), 'g', 'LineWidth', 2, 'DisplayName', 'Optimal Race Line');
    scatter(optimizedControlPoints(:,1), optimizedControlPoints(:,2), 50, 'k', 'filled', 'DisplayName', 'Control Points');
    legend('Location', 'southoutside', 'Orientation', 'horizontal');
    title('Soccer Field Racing Track with Optimal Race Line');
    xlabel('X');
    ylabel('Y');
    axis equal;

    % Dynamically adjust plot limits based on data
    xAll = [leftBoundary(:,1); rightBoundary(:,1); raceLine(:,1)];
    yAll = [leftBoundary(:,2); rightBoundary(:,2); raceLine(:,2)];
    xlim([min(xAll) - 10, max(xAll) + 10]); % Add a margin for better visibility
    ylim([min(yAll) - 10, max(yAll) + 10]); % Add a margin for better visibility

    hold off;
end

% Function to define the soccer field track boundaries
function [leftBoundary, rightBoundary] = soccer_field_boundaries_corrected()
    % Parameters
    straightLength = 100; % Length of straight sections
    radius = 30; % Radius of rounded ends
    nPointsArc = 100; % Number of points for the arcs
    nPointsStraight = 100; % Number of points for the straight sections

    % Generate left boundary
    leftBoundary = [];
    % Semi-circle for the left rounded corner
    theta = linspace(pi/2, 3*pi/2, nPointsArc)';
    leftBoundary = [radius * cos(theta) - straightLength/2, radius * sin(theta)];
    % Straight section along the bottom
    leftBoundary = [leftBoundary; ...
                    linspace(-straightLength/2, straightLength/2, nPointsStraight)', -radius * ones(nPointsStraight, 1)];
    % Semi-circle for the right rounded corner
    theta = linspace(-pi/2, pi/2, nPointsArc)';
    leftBoundary = [leftBoundary; ...
                    radius * cos(theta) + straightLength/2, radius * sin(theta)];
    % Straight section along the top
    leftBoundary = [leftBoundary; ...
                    linspace(straightLength/2, -straightLength/2, nPointsStraight)', radius * ones(nPointsStraight, 1)];

    % Generate right boundary (inner track)
    offset = 10; % Offset for the inner track
    rightBoundary = [];
    % Semi-circle for the left rounded corner
    theta = linspace(pi/2, 3*pi/2, nPointsArc)';
    rightBoundary = [(radius - offset) * cos(theta) - straightLength/2, (radius - offset) * sin(theta)];
    % Straight section along the bottom
    rightBoundary = [rightBoundary; ...
                     linspace(-straightLength/2, straightLength/2, nPointsStraight)', -(radius - offset) * ones(nPointsStraight, 1)];
    % Semi-circle for the right rounded corner
    theta = linspace(-pi/2, pi/2, nPointsArc)';
    rightBoundary = [rightBoundary; ...
                     (radius - offset) * cos(theta) + straightLength/2, (radius - offset) * sin(theta)];
    % Straight section along the top
    rightBoundary = [rightBoundary; ...
                     linspace(straightLength/2, -straightLength/2, nPointsStraight)', (radius - offset) * ones(nPointsStraight, 1)];
end

% Function to generate Bezier curve
function curve = bezier_curve(controlPoints, nPoints)
    n = size(controlPoints, 1) - 1;
    t = linspace(0, 1, nPoints)';
    curve = zeros(nPoints, 2);
    for i = 0:n
        bernstein = nchoosek(n, i) .* (1 - t).^(n - i) .* t.^i;
        curve = curve + bernstein * controlPoints(i + 1, :);
    end
end

% Cost function for optimization
function cost = cost_function(controlPoints, leftBoundary, rightBoundary)
    try
        % Generate Bezier curve
        curve = bezier_curve(controlPoints, 1000);

        % Interpolate centerline to match curve resolution
        centerLine = (leftBoundary + rightBoundary) / 2;
        centerLineInterp = interp1(1:size(centerLine, 1), centerLine, ...
                                   linspace(1, size(centerLine, 1), size(curve, 1)), 'linear');

        % Penalize deviation from the centerline
        distToCenter = sum(vecnorm(curve - centerLineInterp, 2, 2));

        % Calculate curvature
        dCurve = gradient(curve);
        ddCurve = gradient(dCurve);
        curvature = sum(vecnorm(ddCurve, 2, 2));

        % Combine into a scalar cost
        cost = 0.6 * distToCenter + 0.4 * curvature;

    catch ME
        fprintf('Error in cost function:\n');
        rethrow(ME);
    end
end
