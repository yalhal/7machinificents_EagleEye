function [lat_deg, lon_deg, alt_m] = ecef2lla(ecef_m)
    % Calculate latitude, longitude and altitude from a position vector in the ECEF coordinate system
    % IMPORTANT - ASSUMING A SPHERICAL EARTH
    %
    % Input:
    %   ecef_m: position vector in ECEF (3 x 1) [m]
    %
    % Outputs:
    %   lat_deg: latitude (scalar) [deg]
    %   lon_deg: longitude (scalar) [deg]
    %   alt_m: altitude (scalar) [m]

    global r_earth_

    x = ecef_m(1);
    y = ecef_m(2);
    z = ecef_m(3);

    lon_rad = atan2(y, x);
    rho = sqrt(x.^2 + y.^2);
    lat_rad = atan2(z, rho);
    r = sqrt(x.^2 + y.^2 + z.^2);
    alt_m = r - r_earth_;
    lat_deg = rad2deg(lat_rad);
    lon_deg = rad2deg(lon_rad);
end