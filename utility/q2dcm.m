function dcm = q2dcm(q)
    % Calculate dcm from quaternion
    % 
    % Input:
    %   q: quaternion (4 x 1, q(4) is scalar part)
    %
    % Output:
    %   dcm: Direction Cosine Matrix (3 x 3)

    dcm = [
        q(1)^2-q(2)^2-q(3)^2+q(4)^2, 2*(q(1)*q(2)+q(3)*q(4))    , 2*(q(3)*q(1)-q(2)*q(4));
        2*(q(1)*q(2)-q(3)*q(4))    , q(2)^2-q(3)^2-q(1)^2+q(4)^2, 2*(q(2)*q(3)+q(1)*q(4));
        2*(q(3)*q(1)+q(2)*q(4))    , 2*(q(2)*q(3)-q(1)*q(4))    , -q(1)^2-q(2)^2+q(3)^2+q(4)^2;
    ];
end