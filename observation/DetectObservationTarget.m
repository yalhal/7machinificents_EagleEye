function [index, los, pxpy, mxpy, mxmy, pxmy, proportion_before, proportion_after] = DetectObservationTarget()

    global targets_ observation_results_
    global TARGET_NE_INDEX TARGET_NW_INDEX TARGET_SE_INDEX
    global OBSERVATION_RESULT_ALL_AREA_INDEX OBSERVATION_RESULT_OBSERVED_AREA_INDEX

    [los, pxpy, mxpy, mxmy, pxmy] = CalcObservedPoints();
    points = [los; pxpy; mxpy; mxmy; pxmy];
    for i=1:length(targets_)
        target = targets_{i};
        lat_upper = target{TARGET_NE_INDEX}(1);
        lat_lower = target{TARGET_SE_INDEX}(1);
        lon_upper = target{TARGET_NE_INDEX}(2);
        lon_lower = target{TARGET_NW_INDEX}(2);
        for j=1:length(points)
            lat = points(j, 1);
            lon = points(j, 2);
            if lat < lat_lower || lat_upper < lat
                continue
            end
            if lon < lon_lower || lon_upper < lon
                continue
            end
            observed = observation_results_{i};
            index = i;
            poly = polyshape(points(2:5,2), points(2:5,1));
            new_area = intersect(observed{OBSERVATION_RESULT_ALL_AREA_INDEX}, poly);
            conbined_area = union(observed{OBSERVATION_RESULT_OBSERVED_AREA_INDEX}, new_area);
            proportion_before = area(observed{OBSERVATION_RESULT_OBSERVED_AREA_INDEX}) / area(observed{OBSERVATION_RESULT_ALL_AREA_INDEX});
            proportion_after = area(conbined_area) / area(observed{OBSERVATION_RESULT_ALL_AREA_INDEX});
            return
        end
    end

    % NO TARGET IN FOV
    index = 0;
    los = [NaN, NaN];
    pxpy = [NaN, NaN];
    mxpy = [NaN, NaN];
    mxmy = [NaN, NaN];
    pxmy = [NaN, NaN];
    proportion_before = NaN;
    proportion_after = NaN;

end
