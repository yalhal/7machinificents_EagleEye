function [] = UpdateStateGraph(T_rw, M_mtq)
    global T_rw_bar_ M_mtq_bar_ hw_bar_ total_h_bar_
    global x_ N_hw N_w II_ conv_rw2body_
    T_rw_bar_.YData = T_rw;
    M_mtq_bar_.YData = M_mtq;
    hw = x_(N_hw);
    hw_bar_.YData = hw;
    w = x_(N_w);
    h = cross(w, II_ * w + conv_rw2body_ * hw);
    total_h_bar_.YData = h;
end
