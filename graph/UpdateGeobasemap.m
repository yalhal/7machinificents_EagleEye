function [] = UpdateGeobasemap(lat, lon, updated_target_index)
    global map_ax_ map_satellite_marker_
    global targets_ TARGET_CENTER_INDEX
    global observation_results_ OBSERVATION_RESULT_MAXIMUM_SCORE_INDEX OBSERVATION_RESULT_CURRENT_SCORE_INDEX

    % geoplot(map_ax_, lat, lon, "ko");
    map_satellite_marker_.LatitudeData = lat;
    map_satellite_marker_.LongitudeData = lon;
    if updated_target_index ~= 0
        result = observation_results_{updated_target_index};
        target = targets_{updated_target_index};
        target_lat = target{TARGET_CENTER_INDEX}(1);
        target_lon = target{TARGET_CENTER_INDEX}(2);
        prop = result{OBSERVATION_RESULT_CURRENT_SCORE_INDEX} / result{OBSERVATION_RESULT_MAXIMUM_SCORE_INDEX};
        if prop > 0.8
            geoscatter(map_ax_, target_lat, target_lon, "r");
        elseif prop > 0.5
            geoscatter(map_ax_, target_lat, target_lon, "g");
        end
        total_score = 0;
        for i=1:length(observation_results_)
            total_score = total_score + observation_results_{i}{OBSERVATION_RESULT_CURRENT_SCORE_INDEX};
        end
        title(map_ax_, sprintf("Total Score = %.1f", total_score));
    end
    geolimits(map_ax_, [30, 60], [130, 150]);

end
