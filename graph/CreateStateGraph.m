function [] = CreateStateGraph()
    global T_rw_bar_ M_mtq_bar_ hw_bar_ total_h_bar_
    global x_ N_hw N_w II_ conv_rw2body_ trq_max_ mtq_max_ hw_max_
    figure;
    subplot(4, 1, 1);
    T_rw_bar_ = bar([0, 0, 0, 0]);
    ylim([-trq_max_, trq_max_])
    ylabel("RW Torque [Nm]");
    xticklabels(["X", "Y", "Z", "S"]);
    subplot(4, 1, 2);
    hw = x_(N_hw);
    hw_bar_ = bar(hw);
    ylim([-hw_max_, hw_max_])
    ylabel("RW Moment [Nms]");
    xticklabels(["X", "Y", "Z", "S"]);
    subplot(4, 1, 3);
    M_mtq_bar_ = bar([0, 0, 0]);
    ylim([-mtq_max_, mtq_max_])
    ylabel("MTQ Output [Am2]");
    xticklabels(["X", "Y", "Z"]);
    subplot(4, 1, 4);
    w = x_(N_w);
    h = cross(w, II_ * w + conv_rw2body_ * hw);
    total_h_bar_ = bar(h);
    ylabel("Total Moment [Nms]");
    xticklabels(["X", "Y", "Z"]);
end
