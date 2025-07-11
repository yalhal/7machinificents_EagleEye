function [q_out, w_out, h_out] = AttitudeUpdate(q_in, w_in, h_in, dt, II, IIinv, T_rw, conv_rw2body_, T_other)
    [q_k1, w_k1, h_k1] = dx(q_in, w_in, h_in, II, IIinv,  T_rw, conv_rw2body_, T_other);
    [q_k2, w_k2, h_k2] = dx(q_in + q_k1 * dt / 2, w_in + w_k1 * dt / 2, h_in + h_k1 * dt / 2, II, IIinv,  T_rw, conv_rw2body_, T_other);
    [q_k3, w_k3, h_k3] = dx(q_in + q_k2 * dt / 2, w_in + w_k2 * dt / 2, h_in + h_k2 * dt / 2, II, IIinv,  T_rw, conv_rw2body_, T_other);
    [q_k4, w_k4, h_k4] = dx(q_in + q_k3 * dt, w_in + w_k3 * dt, h_in + h_k3 * dt, II, IIinv,  T_rw, conv_rw2body_, T_other);
    q_out = q_in + (dt / 6) * (q_k1 + 2 * q_k2 + 2 * q_k3 + q_k4);
    w_out = w_in + (dt / 6) * (w_k1 + 2 * w_k2 + 2 * w_k3 + w_k4);
    h_out = h_in + (dt / 6) * (h_k1 + 2 * h_k2 + 2 * h_k3 + h_k4);
    % normalize
    q_out = normalize(q_out, "norm");
    if q_out(4) < 0
        q_out = -q_out;
    end
end

function [dq, dw, dh] = dx(q, w, hw, II, IIinv, T_rw, conv_rw2body_, T_other)
    wmat = [
          0.0,  w(3), -w(2), w(1);
        -w(3),   0.0,  w(1), w(2);
         w(2), -w(1),   0.0, w(3);
        -w(1), -w(2), -w(3),  0.0;
    ];
    dq = 0.5 * wmat * q;
    T_all = -conv_rw2body_ * T_rw + T_other;
    dw = IIinv * (-cross(w, II * w + conv_rw2body_ * hw) + T_all);
    dh = T_rw;
end
