function M_mtq_out = CheckMtqConstraint(M_mtq_in)
    global mtq_max_
    M_mtq_out = M_mtq_in;
    for i = 1:length(M_mtq_in)
        M = M_mtq_in(i);
        if abs(M) > mtq_max_
            M = sign(M) * mtq_max_;
        end
        M_mtq_out(i) = M;
    end
end