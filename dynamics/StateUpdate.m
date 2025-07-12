function [] = StateUpdate(air_rho, mag_field_ecef, T_rw, M_mtq)
    global shapes_ x_ dth_ N_r N_v N_q N_w N_hw Cd_ mass_ II_ II_inv_ mu_ conv_rw2body_ utc_

    r = x_(N_r);
    v = x_(N_v);
    q = x_(N_q);
    w = x_(N_w);
    hw = x_(N_hw);

    % air
    [projected_area, arm] = ProjectedArea(shapes_, v);
    air_force = -0.5 * air_rho * Cd_ * projected_area * norm(v) * v;
    air_acc = air_force / mass_;
    air_torque = cross(arm, air_force);
    % display(air_force)
    % display(arm)
    % display(air_torque)

    % mag
    mag_field_body = q2dcm(q) * ecef2eci(mag_field_ecef, utc_);
    mag_torque = cross(M_mtq, mag_field_body);

    [r_out, v_out] = OrbitUpdate(r, v, dth_, mu_, air_acc);
    [q_out, w_out, h_out] = AttitudeUpdate(q, w, hw, dth_, II_, II_inv_, T_rw, conv_rw2body_, mag_torque + air_torque);

    x_(N_r) = r_out;
    x_(N_v) = v_out;
    x_(N_q) = q_out;
    x_(N_w) = w_out;
    x_(N_hw) = h_out;

end