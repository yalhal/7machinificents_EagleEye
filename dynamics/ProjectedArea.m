function [projected_area, arm] = ProjectedArea(faces, projection_vector)
    % Calculate projected area of faces in the direction of projection_vector
    %
    % Inputs:
    %   faces: Definition of faces (n x 1 cell array. each element is list of vertices (3 x m)).
    %   projection_vector: Direction of projection (3 x 1).
    %
    % Returns:
    %   projected_area: Total area of projected faces (scalar).
    %   arm: Vector from origin to centroid of projected area (3 x 1).

    d = normalize(projection_vector, "norm");
    ang1 = atan2(d(2), d(1));
    ang2 = -atan2(d(3), sqrt(d(1)^2 + d(2)^2));
    dcm1 = CoordinateRotation("z", ang1);
    dcm2 = CoordinateRotation("y", ang2);
    dcm = dcm2 * dcm1;

    all_shapes = polyshape();
    for i = 1:length(faces)
        points = dcm * faces{i};
        poly = polyshape(points(2,:), points(3,:));
        all_shapes = union(all_shapes, poly);
    end

    projected_area = area(all_shapes);
    [y, z] = centroid(all_shapes);
    arm = dcm1' * dcm2' * [0; y; z];
end