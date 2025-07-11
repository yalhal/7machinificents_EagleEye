function [] = UpdateObservedAreaPlot(index)
    global observation_results_
    global current_plot_target_index_
    global observation_area_plot_axes_

    global OBSERVATION_RESULT_NAME_INDEX OBSERVATION_RESULT_ALL_AREA_INDEX OBSERVATION_RESULT_OBSERVED_AREA_INDEX
    global OBSERVATION_RESULT_CURRENT_SCORE_INDEX

    if current_plot_target_index_ ~= index
        current_plot_target_index_ = index;
        figure;
        observation_area_plot_axes_ = gca;
    end
    result = observation_results_{index};
    hold on;
    % plot(observation_area_plot_axes_, result{OBSERVATION_RESULT_ALL_AREA_INDEX}, "FaceColor", "none");
    plot(result{OBSERVATION_RESULT_ALL_AREA_INDEX}, "FaceColor", "none");
    % plot(observation_area_plot_axes_, result{OBSERVATION_RESULT_OBSERVED_AREA_INDEX});
    plot(result{OBSERVATION_RESULT_OBSERVED_AREA_INDEX});
    percentage = 100 * area(result{OBSERVATION_RESULT_OBSERVED_AREA_INDEX}) / area(result{OBSERVATION_RESULT_ALL_AREA_INDEX});
    % percentage = 100 * result{OBSERVATION_RESULT_CURRENT_SCORE_INDEX} / result{OBSERVATION_RESULT_MAXIMUM_SCORE_INDEX};
    score = result{OBSERVATION_RESULT_CURRENT_SCORE_INDEX};
    title(observation_area_plot_axes_, sprintf("%s Score: %.1f (%.1f %%)", result{OBSERVATION_RESULT_NAME_INDEX}, score, percentage));

end
