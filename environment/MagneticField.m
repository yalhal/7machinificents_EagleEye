function B_ecef = MagneticField(lat_deg, lon_deg, alt_m)
    % SIMPLE DIPOLE MODEL

    global r_earth_

    % Equatorial surface field strength [T]
    B0 = 3.12e-5;

    % Dipole tilt direction in ECEF (unit vector)
    tilt = deg2rad(9.2);
    azimuth = deg2rad(252.8);
    m_ecef = [cos(tilt)*cos(azimuth); cos(tilt)*sin(azimuth); sin(tilt)];

    r_ecef = lla2ecef(lat_deg, lon_deg, alt_m);
    r = norm(r_ecef);
    r_hat = r_ecef / r;

    % Dipole magnetic field formula in ECEF
    B_ecef = B0 * (r_earth_ / r)^3 * (3 * dot(m_ecef, r_hat) * r_hat - m_ecef);
end
