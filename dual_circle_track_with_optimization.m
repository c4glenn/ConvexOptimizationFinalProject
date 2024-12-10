function flower_shape_3D_advanced()
    % Define parameters
    R_outer = 20; % Radius of the outer circles
    R_inner = 10; % Radius of the inner circle
    straight_length = 30; % Length of the straight sections
    num_points_circle = 100; % Number of points for the circular sections
    num_points_straight = 50; % Number of points for the straight sections

    % Generate track boundaries
    [leftBoundary, rightBoundary] = generate_3D_flower_shape_track(R_outer, R_inner, straight_length, num_points_circle, num_points_straight);

    % Initial control points
    numControlPoints = 10; % Number of Bezier control points
    initialControlPoints = initialize_control_points(leftBoundary, rightBoundary, numControlPoints);

    % Optimize the control points
    options = optimoptions('fmincon', 'Display', 'iter', 'Algorithm', 'sqp', ...
                           'MaxFunctionEvaluations', 5000, 'StepTolerance', 1e-9);
    optimizedControlPoints = fmincon(@(x) cost_function(reshape(x, [], 3), leftBoundary, rightBoundary), ...
                                     initialControlPoints(:), [], [], [], [], [], [], [], options);

    % Reshape optimized control points
    optimizedControlPoints = reshape(optimizedControlPoints, [], 3);

    % Generate the Bezier curve
    raceLine = bezier_curve(optimizedControlPoints, 300);

    % Plot the track and race line in 3D
    figure;
    hold on;
    plot3(leftBoundary(:,1), leftBoundary(:,2), leftBoundary(:,3), 'r', 'LineWidth', 2, 'DisplayName', 'Left Boundary');
    plot3(rightBoundary(:,1), rightBoundary(:,2), rightBoundary(:,3), 'b', 'LineWidth', 2, 'DisplayName', 'Right Boundary');
    plot3(raceLine(:,1), raceLine(:,2), raceLine(:,3), 'g', 'LineWidth', 2, 'DisplayName', 'Optimal Race Line');
    scatter3(optimizedControlPoints(:,1), optimizedControlPoints(:,2), optimizedControlPoints(:,3), 50, 'k', 'filled', 'DisplayName', 'Control Points');
    legend('Location', 'best');
    title('Advanced 3D Flower Shape Racing Track with Bezier Curve Optimization');
    xlabel('X');
    ylabel('Y');
    zlabel('Elevation (Z)');
    grid on;
    axis equal;
    view(3); % 3D view
    hold off;
end

function [leftBoundary, rightBoundary] = generate_3D_flower_shape_track(R_outer, R_inner, straight_length, num_points_circle, num_points_straight)
    % Generate circular and straight sections with elevation
    theta = linspace(0, pi, num_points_circle)';
    
    % Left circle
    left_circle_outer = [-R_outer * cos(theta), R_outer * sin(theta), zeros(num_points_circle, 1)];
    left_circle_inner = [-R_inner * cos(theta), R_inner * sin(theta), zeros(num_points_circle, 1)];

    % Right circle
    right_circle_outer = [R_outer * cos(theta), -R_outer * sin(theta), zeros(num_points_circle, 1)];
    right_circle_inner = [R_inner * cos(theta), -R_inner * sin(theta), zeros(num_points_circle, 1)];

    % Middle semi-circle
    middle_circle_outer = [R_outer * cos(theta), R_outer * sin(theta), linspace(0, 20, num_points_circle)'];
    middle_circle_inner = [R_inner * cos(theta), R_inner * sin(theta), linspace(0, 20, num_points_circle)'];

    % Straight sections
    straight_elevation = linspace(0, 10, num_points_straight)';
    straight1_outer = [linspace(-R_outer, R_outer, num_points_straight)', zeros(num_points_straight, 1), straight_elevation];
    straight1_inner = [linspace(-R_inner, R_inner, num_points_straight)', zeros(num_points_straight, 1), straight_elevation];

    % Combine boundaries
    leftBoundary = [left_circle_outer; straight1_outer; middle_circle_outer];
    rightBoundary = [left_circle_inner; straight1_inner; middle_circle_inner];
end

function controlPoints = initialize_control_points(leftBoundary, rightBoundary, numControlPoints)
    controlPoints = zeros(numControlPoints, 3);
    for i = 1:numControlPoints
        idx = round((i - 1) * (size(leftBoundary, 1) - 1) / (numControlPoints - 1)) + 1;
        controlPoints(i, :) = (leftBoundary(idx, :) + rightBoundary(idx, :)) / 2;
    end
end

function curve = bezier_curve(controlPoints, nPoints)
    n = size(controlPoints, 1) - 1;
    t = linspace(0, 1, nPoints)';
    curve = zeros(nPoints, 3);
    for i = 0:n
        bernstein = nchoosek(n, i) .* (1 - t).^(n - i) .* t.^i;
        curve = curve + bernstein * controlPoints(i + 1, :);
    end
end

function cost = cost_function(controlPoints, leftBoundary, rightBoundary)
    curve = bezier_curve(controlPoints, 300);

    % Distance to boundaries
    distToLeft = min(pdist2(curve, leftBoundary), [], 2);
    distToRight = min(pdist2(curve, rightBoundary), [], 2);

    % Centerline
    centerLine = (leftBoundary + rightBoundary) / 2;
    centerLineInterp = interp1(1:size(centerLine, 1), centerLine, linspace(1, size(centerLine, 1), size(curve, 1)));

    % Deviation from centerline
    distToCenter = sum(vecnorm(curve - centerLineInterp, 2, 2));

    % Curvature
    dCurve = gradient(curve);
    ddCurve = gradient(dCurve);
    curvature = sum(vecnorm(ddCurve, 2, 2));

    cost = 0.5 * curvature + 0.5 * distToCenter;
end
