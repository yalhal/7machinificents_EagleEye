function [] = Observe(index, points)
    global t_ t_last_observed_
    global observation_results_ observation_cover_min_
    global OBSERVATION_RESULT_ALL_AREA_INDEX OBSERVATION_RESULT_OBSERVED_AREA_INDEX OBSERVATION_RESULT_MAXIMUM_SCORE_INDEX OBSERVATION_RESULT_CURRENT_SCORE_INDEX
    poly = polyshape(points(:,2), points(:,1));
    new_area = intersect(observation_results_{index}{OBSERVATION_RESULT_ALL_AREA_INDEX}, poly);
    observation_results_{index}{OBSERVATION_RESULT_OBSERVED_AREA_INDEX} = union(observation_results_{index}{OBSERVATION_RESULT_OBSERVED_AREA_INDEX}, new_area);
    prop = area(observation_results_{index}{OBSERVATION_RESULT_OBSERVED_AREA_INDEX}) / area(observation_results_{index}{OBSERVATION_RESULT_ALL_AREA_INDEX});
    if prop > observation_cover_min_
        c = (prop - observation_cover_min_) / (1 - observation_cover_min_);
    else
        c = 0;
    end
    observation_results_{index}{OBSERVATION_RESULT_CURRENT_SCORE_INDEX} = observation_results_{index}{OBSERVATION_RESULT_MAXIMUM_SCORE_INDEX} * c;
    t_last_observed_ = t_;
end
