function R = CoordinateRotation(axis, angle, unit)
    % Calculate DCM for coordinate rotation
    % 
    % Inputs:
    %   axis: Character 'x', 'y', or 'z' specifying the rotation axis
    %   angle: Rotation angle (in radians or degrees)
    %   unit: (Optional) 'rad' or 'deg'. Defaults to 'rad'
    %
    % Output:
    %   R: Direction Cosine Matrix (3 x 3)

    if nargin < 3
        unit = 'rad';
    end

    if strcmpi(unit, 'deg')
        angle = deg2rad(angle);
    end

    c = cos(angle);
    s = sin(angle);

    switch lower(axis)
        case 'x'
            R = [1   0  0;
                 0   c  s;
                 0  -s  c];
        case 'y'
            R = [ c  0 -s;
                  0  1  0;
                  s  0  c];
        case 'z'
            R = [ c  s  0;
                 -s  c  0;
                  0  0  1];
        otherwise
            error('Axis must be one of ''x'', ''y'', or ''z''.');
    end
end