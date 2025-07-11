function [los, pxpy, mxpy, mxmy, pxmy] = CalcObservedPoints()
    % calculate the points where los_ and corners of FOV intersect the surface of the earth
    %      los: the point where los_ intersects the surface of the earth [lat_deg, lon_deg]
    %      pxpy: the point where fov_corner_pxpy intersects the surface of the earth [lat_deg, lon_deg]
    %      mxpy: same as pxpy
    %      mxmy: same as pxpy
    %      pxmy: same as pxpy
    global utc_ x_ N_r N_q
    global los_ fov_corners_
    global FOV_CORNER_PXPY_INDEX FOV_CORNER_MXPY_INDEX FOV_CORNER_MXMY_INDEX FOV_CORNER_PXMY_INDEX
    r_eci = x_(N_r);
    q = x_(N_q);
    dcm = q2dcm(q)';
    los_eci = dcm * los_;
    corner_pxpy_eci = dcm * fov_corners_{FOV_CORNER_PXPY_INDEX};
    corner_mxpy_eci = dcm * fov_corners_{FOV_CORNER_MXPY_INDEX};
    corner_mxmy_eci = dcm * fov_corners_{FOV_CORNER_MXMY_INDEX};
    corner_pxmy_eci = dcm * fov_corners_{FOV_CORNER_PXMY_INDEX};
    [los_lat, los_lon, ~] = ecef2lla(eci2ecef(CalcIntersectPoint_(r_eci, los_eci), utc_));
    [pxpy_lat, pxpy_lon, ~] = ecef2lla(eci2ecef(CalcIntersectPoint_(r_eci, corner_pxpy_eci), utc_));
    [mxpy_lat, mxpy_lon, ~] = ecef2lla(eci2ecef(CalcIntersectPoint_(r_eci, corner_mxpy_eci), utc_));
    [mxmy_lat, mxmy_lon, ~] = ecef2lla(eci2ecef(CalcIntersectPoint_(r_eci, corner_mxmy_eci), utc_));
    [pxmy_lat, pxmy_lon, ~] = ecef2lla(eci2ecef(CalcIntersectPoint_(r_eci, corner_pxmy_eci), utc_));
    los = [los_lat, los_lon];
    pxpy = [pxpy_lat, pxpy_lon];
    mxpy = [mxpy_lat, mxpy_lon];
    mxmy = [mxmy_lat, mxmy_lon];
    pxmy = [pxmy_lat, pxmy_lon];
end

function [p_eci] = CalcIntersectPoint_(r_eci, los_eci)
    global r_earth_
    b = dot(r_eci, los_eci);
    c = norm(r_eci) ^ 2 - r_earth_ ^ 2;
    s = -b - sqrt(b^2 - c);
    p_eci = r_eci + s * los_eci;
end
