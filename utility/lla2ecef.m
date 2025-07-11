function ecef_m = lla2ecef(lat_deg, lon_deg, alt_m)
    % Calculate a position vector in the ECEF coordinate system from latitude, longitude and altitude
    % IMPORTANT - ASSUMING A SPHERICAL EARTH
    %
    % Inputs:
    %   lat_deg: latitude (scalar) [deg]
    %   lon_deg: longitude (scalar) [deg]
    %   alt_m: altitude (scalar) [m]
    %
    % Output:
    %   ecef_m: position vector in ECEF (3 x 1) [m]

    global r_earth_

    lat_rad = deg2rad(lat_deg);
    lon_rad = deg2rad(lon_deg);
    r = r_earth_ + alt_m;
    x = r .* cos(lat_rad) .* cos(lon_rad);
    y = r .* cos(lat_rad) .* sin(lon_rad);
    z = r .* sin(lat_rad);
    ecef_m = [x; y; z];
end