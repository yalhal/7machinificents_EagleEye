function [is_valid] = IsSatisfyOffNadirRequirement()
    % Check whether the current off-nadir angle satisfies the requirements
    %
    % Output:
    %   is_valid: whether the current off-nadir angle satisfies the requirements (scalar, bool)

    global los_ observation_offnadir_max_
    global x_ N_r N_q 

    r = x_(N_r);
    q = x_(N_q);
    nadir_body = -normalize(q2dcm(q) * r, "norm");
    offnadir_angle = acos(dot(nadir_body, los_));
    if offnadir_angle > observation_offnadir_max_
        is_valid = false;
    else
        is_valid = true;
    end
end
