function rho = AirDensity(alt_m)
    % SIMPLE IMPLEMENTATION BASED ON ALTITUDE
    h0_arr = [150, 180, 200, 250, 300, 350, 400, 450, 500];
    H_arr = [22.523, 29.740, 37.105, 45.546, 53.628, 53.298, 58.515, 60.828, 63.822];
    rho0_arr = [2.070e-9, 5.464e-10, 2.789e-10, 7.248e-11, 2.418e-11, 9.158e-12, 3.725e-12, 1.585e-12, 6.967e-13];
    alt_km = alt_m / 1000;
    index = length(h0_arr);
    for i = 2:length(h0_arr)
        h0 = h0_arr(i);
        if alt_km < h0
            index = i - 1;
            break
        end
    end
    h0 = h0_arr(index);
    H = H_arr(index);
    rho0 = rho0_arr(index);
    rho = rho0 * exp(-(alt_km - h0)/H);
end