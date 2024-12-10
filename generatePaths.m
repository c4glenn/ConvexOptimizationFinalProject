function [innerPath, outerPath] = generatePaths(imageFile, spacing)
        % Read the input image
    path = skeletonToLoopPath(imageFile);

    [innerPath, outerPath] = offsetPaths(path, spacing);
    
    % %Visualize the skeleton
    % figure;
    % img = imread(imageFile);
    % imshow(img);
    % hold on;
    % plot(path(:, 1), path(:, 2), 'r-', 'MarkerSize', 5);
    % plot(innerPath(:, 1), innerPath(:, 2), 'b-', 'LineWidth', 2); % Path 1
    % plot(outerPath(:, 1), outerPath(:, 2), 'g-', 'LineWidth', 2); % Path 2
    % 
    % title('Skeleton of the Path');
    % hold off;

end

function orderedPath = skeletonToLoopPath(imagePath)
    % Read the input image
    img = imread(imagePath);
    flippedImg = flipud(img); % Flip the image vertically

    % Convert the image to grayscale
    grayImg = rgb2gray(img);
    
    % Threshold the grayscale image to create a binary mask
    binaryMask = grayImg < 50; % Adjust threshold if needed
    
    % Skeletonize the path to get a single-pixel-wide line
    skeleton = bwmorph(binaryMask, 'skel', Inf);
    skeleton = imclose(skeleton, strel('disk', 1));

    % Find coordinates of the skeleton
    [rows, cols] = find(skeleton);
    skeletonPoints = [cols, rows];
    
    % If no skeleton points are found, return an empty result
    if isempty(skeletonPoints)
        warning('No skeleton points found in the image.');
        orderedPath = [];
        return;
    end
    
    % Create a connectivity matrix to define neighbors
    [height, width] = size(skeleton);
    connectivityOffsets = [
        -1, -1; -1, 0; -1, 1;
         0, -1;        0, 1;
         1, -1;  1, 0;  1, 1
    ];
    
    % Start ordering from any point
    orderedPath = skeletonPoints(1, :);
    remainingPoints = skeletonPoints(2:end, :);
    
    % Sequentially order the points
    while ~isempty(remainingPoints)
        lastPoint = orderedPath(end, :);
        
        % Find neighbors of the last point
        neighbors = lastPoint + connectivityOffsets;
        
        % Check which neighbors are in the remaining points
        [isNeighbor, neighborIdx] = ismember(remainingPoints, neighbors, 'rows');
        
        if any(isNeighbor)
            % Add the first valid neighbor to the path
            nextIdx = find(isNeighbor, 1, 'first');
            nextPoint = remainingPoints(nextIdx, :);
            orderedPath = [orderedPath; nextPoint];
            % Remove the added point from the remaining points
            remainingPoints(nextIdx, :) = [];
        else
            % If no neighbors are found, the path is complete
            break;
        end
    end
    
    % Close the loop by adding the first point to the end
    orderedPath = [orderedPath; orderedPath(1, :)];
    
    orderedPath = smoothPath(orderedPath);


    % % Visualize the ordered loop path
    % figure;
    % imshow(img);
    % hold on;
    % plot(orderedPath(:, 1), orderedPath(:, 2), 'r-', 'LineWidth', 2);
    % title('Ordered Loop Skeleton Path');
    % hold off;
end


function smoothedPath = smoothPath(path)
    % Use Savitzky-Golay filter to smooth the x and y coordinates
    windowSize = 9; % Must be odd
    polynomialOrder = 6; % Polynomial order for smoothing
    smoothedX = sgolayfilt(path(:, 1), polynomialOrder, windowSize);
    smoothedY = sgolayfilt(path(:, 2), polynomialOrder, windowSize);
    smoothedPath = [smoothedX, smoothedY];
end


function [path1, path2] = offsetPaths(centerline, spacing)
    % Compute perpendicular offsets for each point in the centerline
    path1 = zeros(size(centerline));
    path2 = zeros(size(centerline));

    for i = 2:size(centerline, 1)-1
        % Calculate tangent vector
        tangent = centerline(i+1, :) - centerline(i-1, :);
        tangent = tangent / norm(tangent); % Normalize

        % Calculate perpendicular vector
        perp = [-tangent(2), tangent(1)];

        % Offset paths
        path1(i, :) = centerline(i, :) + spacing * perp;
        path2(i, :) = centerline(i, :) - spacing * perp;
    end
    
    % Fill in endpoints manually
    path1(1, :) = path1(2, :);
    path1(end, :) = path1(end-1, :);
    path2(1, :) = path2(2, :);
    path2(end, :) = path2(end-1, :);
end
