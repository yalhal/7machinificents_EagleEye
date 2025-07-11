function T_rw_out = CheckRwConstraint(T_rw_in)
    global trq_max_ hw_max_ x_ N_hw
    T_rw_out = T_rw_in;
    h_rw = x_(N_hw);
    for i = 1:length(T_rw_in)
        T = T_rw_in(i);
        hw = h_rw(i);
        % torque limit
        if abs(T) > trq_max_
            T = sign(T) * trq_max_;
        end
        % momentum limit
        if abs(hw) >= hw_max_
            if sign(T) == sign(hw)
                T = 0;
            end
        end
        T_rw_out(i) = T;
    end
end