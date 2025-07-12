function [] = set_InitialValues

    global x_ x_prev_ utc_ utc_prev_ t_last_observed_
    global N_sv N_r N_v N_q N_w N_hw
    global targets_ observation_results_
    global TARGET_NAME_INDEX TARGET_CENTER_INDEX TARGET_NE_INDEX TARGET_NW_INDEX TARGET_SW_INDEX TARGET_SE_INDEX TARGET_MAX_SCORE_INDEX
    global OBSERVATION_RESULT_NAME_INDEX OBSERVATION_RESULT_ALL_AREA_INDEX OBSERVATION_RESULT_OBSERVED_AREA_INDEX
    global OBSERVATION_RESULT_MAXIMUM_SCORE_INDEX OBSERVATION_RESULT_CURRENT_SCORE_INDEX
    global current_plot_target_index_

    utc_ = datetime(2025, 5, 8, 0, 48, 0);
    x_ = zeros(N_sv, 1);
    x_(N_r) = [3956773.901; 1598570.956; 5136732.910];
    x_(N_v) = [6006.637; 618.309; -4819.272];
    x_(N_q) = [0.826683; 0.358458; 0.413488; 0.130887];
    x_(N_q) = [0; 0.3826834; 0; 0.9238795]; % 追加
    x_(N_w) = [0.0; 0.0; 0.0];
    x_(N_hw) = [0.0; 0.0; 0.0; 0.0];

    utc_prev_ = utc_;
    x_prev_ = x_;
    t_last_observed_ = -Inf;

    OBSERVATION_RESULT_NAME_INDEX = 1;
    OBSERVATION_RESULT_ALL_AREA_INDEX = 2;
    OBSERVATION_RESULT_OBSERVED_AREA_INDEX = 3;
    OBSERVATION_RESULT_MAXIMUM_SCORE_INDEX = 4;
    OBSERVATION_RESULT_CURRENT_SCORE_INDEX = 5;

    observation_results_ = cell(length(targets_), 1);
    figure_lat = zeros(length(targets_), 1);
    figure_lon = zeros(length(targets_), 1);
    for i=1:length(targets_)
        target = targets_{i};
        corners = [
            target{TARGET_NE_INDEX};
            target{TARGET_NW_INDEX};
            target{TARGET_SW_INDEX};
            target{TARGET_SE_INDEX};
        ];
        observation_results_{i} = {
            target{TARGET_NAME_INDEX};             % name
            polyshape(corners(:,2), corners(:,1));  % all area
            polyshape();                            % observed area
            target{TARGET_MAX_SCORE_INDEX};        % maximum score
            0;                                      % current score
        };
        figure_lat(i) = target{TARGET_CENTER_INDEX}(1);
        figure_lon(i) = target{TARGET_CENTER_INDEX}(2);
    end

    % prepare figures
    CreateGeobasemap(figure_lat, figure_lon);
    CreateStateGraph();
    current_plot_target_index_ = 0;
    drawnow;

end