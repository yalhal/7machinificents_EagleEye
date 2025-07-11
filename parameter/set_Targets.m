function [] = set_Targets
    global targets_
    global TARGET_NAME_INDEX TARGET_CENTER_INDEX TARGET_NE_INDEX TARGET_NW_INDEX TARGET_SW_INDEX TARGET_SE_INDEX TARGET_MAX_SCORE_INDEX

    TARGET_NAME_INDEX = 1;
    TARGET_CENTER_INDEX = 2;
    TARGET_NE_INDEX = 3;
    TARGET_NW_INDEX = 4;
    TARGET_SW_INDEX = 5;
    TARGET_SE_INDEX = 6;
    TARGET_MAX_SCORE_INDEX = 7;

    json_targets = jsondecode(fileread("targets.json"));
    targets_ = cell(length(json_targets), 1);
    for i=1:length(json_targets)
        target = json_targets(i);
        targets_{i} = CreateTargetLatLon(target.name, target.center_lat, target.center_lon, target.north_south_km, target.west_east_km);
    end

end

function [target] = CreateTargetLatLon(name, center_lat, center_lon, north_south_km, west_east_km)
    global r_earth_
    dlat = 360 * (north_south_km / 2) / ((r_earth_ / 1000) * 2 * pi);
    dlon = 360 * (west_east_km / 2) / ((r_earth_ / 1000) * 2 * pi);
    max_score = north_south_km * west_east_km;
    target = {
        name;                                    % unique name
        [center_lat, center_lon];                % center
        [center_lat + dlat, center_lon + dlon];  % north-east
        [center_lat + dlat, center_lon - dlon];  % north-west
        [center_lat - dlat, center_lon - dlon];  % south-west
        [center_lat - dlat, center_lon + dlon];  % south-east
        max_score;                               % available maximum score
    };
end
