function [r_out, v_out] = OrbitUpdate(r_in, v_in, dt, mu, a_other)
    [r_k1, v_k1] = dx(r_in, v_in, mu, a_other);
    [r_k2, v_k2] = dx(r_in + r_k1 * dt / 2, v_in + v_k1 * dt / 2, mu, a_other);
    [r_k3, v_k3] = dx(r_in + r_k2 * dt / 2, v_in + v_k2 * dt / 2, mu, a_other);
    [r_k4, v_k4] = dx(r_in + r_k3 * dt, v_in + v_k3 * dt, mu, a_other);
    r_out = r_in + (dt / 6) * (r_k1 + 2 * r_k2 + 2 * r_k3 + r_k4);
    v_out = v_in + (dt / 6) * (v_k1 + 2 * v_k2 + 2 * v_k3 + v_k4);
end

function [r_out, v_out] = dx(r_in, v_in, mu, f_other)
    r_out = v_in;
    r_norm = norm(r_in);
    v_out = -mu * r_in / (r_norm^3) + f_other;
end
