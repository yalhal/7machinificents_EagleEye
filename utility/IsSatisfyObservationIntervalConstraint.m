function [is_valid] = IsSatisfyObservationIntervalConstraint()
    % Check whether the current time has passed the minimum interval since the last observation
    %
    % Output:
    %   is_valid: whether the current time has passed the minimum interval since the last observation (scalar, bool)

    global t_ t_last_observed_ observation_interval_min_

    if observation_interval_min_ > (t_ - t_last_observed_)
        is_valid = false;
    else
        is_valid = true;
    end
end
