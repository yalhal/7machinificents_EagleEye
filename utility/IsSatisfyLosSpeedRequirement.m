function [is_valid] = IsSatisfyLosSpeedRequirement()
    % Check whether the current speed of line of sight on the ground satisfies the requirements
    %
    % Output:
    %   is_valid: whether the current speed of line of sight on the ground satisfies the requirements (scalar, bool)

    global los_ observation_los_speed_max_
    global t_ t_prev_ x_ x_prev_ N_r N_q utc_ utc_prev_
    r = x_(N_r);
    q = x_(N_q);
    r_prev = x_prev_(N_r);
    q_prev = x_prev_(N_q);
    p_now = CalcIntersectPoint(utc_, r, q, los_);
    p_prev = CalcIntersectPoint(utc_prev_, r_prev, q_prev, los_);
    v_los = norm(p_now - p_prev) / (t_ - t_prev_);
    if v_los > observation_los_speed_max_
        is_valid = false;
    else
        is_valid = true;
    end
end
