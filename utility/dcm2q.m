function q = dcm2q(dcm)
    % Calculate quaternion from dcm
    % 
    % Input:
    %   dcm: Direction Cosine Matrix (3 x 3)
    %
    % Output:
    %   q: quaternion (4 x 1, q(4) is scalar part)

    qq = zeros(4, 1);
    q = zeros(4, 1);
    qq(1) = 1.0 / 4.0 * (1.0 + dcm(1, 1) - dcm(2, 2) - dcm(3, 3));
    qq(2) = 1.0 / 4.0 * (1.0 - dcm(1, 1) + dcm(2, 2) - dcm(3, 3));
    qq(3) = 1.0 / 4.0 * (1.0 - dcm(1, 1) - dcm(2, 2) + dcm(3, 3));
    qq(4) = 1.0 / 4.0 * (1.0 + dcm(1, 1) + dcm(2, 2) + dcm(3, 3));
    [~, index] = max(qq);
    if index == 1
        q(1) = sqrt(qq(1));
        q(4) = (dcm(2, 3) - dcm(3, 2)) / 4.0 / q(1);
        q(2) = (dcm(1, 2) + dcm(2, 1)) / 4.0 / q(1);
        q(3) = (dcm(3, 1) + dcm(1, 3)) / 4.0 / q(1);
    elseif index == 2
        q(2) = sqrt(qq(2));
        q(4) = (dcm(3, 1) - dcm(1, 3)) / 4.0 / q(2);
        q(1) = (dcm(1, 2) + dcm(2, 1)) / 4.0 / q(2);
        q(3) = (dcm(2, 3) + dcm(3, 2)) / 4.0 / q(2);
    elseif index == 3
        q(3) = sqrt(qq(3));
        q(4) = (dcm(1, 2) - dcm(2, 1)) / 4.0 / q(3);
        q(1) = (dcm(3, 1) + dcm(1, 3)) / 4.0 / q(3);
        q(2) = (dcm(2, 3) + dcm(3, 2)) / 4.0 / q(3);
    elseif index == 4
        q(4) = sqrt(qq(4));
        q(1) = (dcm(2, 3) - dcm(3, 2)) / 4.0 / q(4);
        q(2) = (dcm(3, 1) - dcm(1, 3)) / 4.0 / q(4);
        q(3) = (dcm(1, 2) - dcm(2, 1)) / 4.0 / q(4);
    end
end