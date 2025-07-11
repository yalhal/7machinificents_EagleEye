function [] = CreateGeobasemap(figure_lat, figure_lon)
    global map_ax_ x_ N_r utc_ map_satellite_marker_

    geobasemap grayland
    geolimits manual
    map_ax_ = gca;
    hold on
    geoscatter(map_ax_, figure_lat, figure_lon, "filled");
    [lat, lon, ~] = ecef2lla(eci2ecef(x_(N_r), utc_));
    map_satellite_marker_ = geoplot(map_ax_, lat, lon, "o", "MarkerEdgeColor", "black");
    geolimits(map_ax_, [30, 55], [130, 145]);
    title(map_ax_, "Total Score = 0.0");

end
