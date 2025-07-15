function [index, los, pxpy, mxpy, mxmy, pxmy, proportion_before, proportion_after] = IsValidObservation()
    % Check whether the target is within the field of view and the observation satisfies the requirements
    %
    % Outputs:
    %    index: target index (if there are no valid target, this is 0.)
    %    los: the point where los_ intersects the surface of the earth ([lat_deg, lon_deg]. if there are no valid target, los is [NaN, NaN].)
    %    pxpy: the point where fov_corner_pxpy intersects the surface of the earth ([lat_deg, lon_deg]. if there are no valid target, los is [NaN, NaN].)
    %    mxpy: same as pxpy
    %    mxmy: same as pxpy
    %    pxmy: same as pxpy
    %    proportion_before: observed proportion of the target (if there are no valid target, this is NaN.)
    %    proportion_after: estimated observed proportion of the target after current observation (if there are no valid target, this is NaN.)

    index = 0;
    los = [NaN, NaN];
    pxpy = [NaN, NaN];
    mxpy = [NaN, NaN];
    mxmy = [NaN, NaN];
    pxmy = [NaN, NaN];
    proportion_before = NaN;
    proportion_after = NaN;

    if (~IsSatisfyOffNadirRequirement())
        fprintf("OffNadirRequirement\n"); % 追加
        return;
    end

    if (~IsSatisfyLosSpeedRequirement())
        fprintf("LosSpeedRequirement\n"); % 追加
        return;
    end

    if (~IsSatisfyObservationIntervalConstraint())
        fprintf("ObservationIntervalConstraint\n"); % 追加
        return;
    end

    [index, los, pxpy, mxpy, mxmy, pxmy, proportion_before, proportion_after] = DetectObservationTarget()

end
