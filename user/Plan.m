function [] = Plan(t, utc, r, v, q, w, hw, targets)
    % user-defined plan algorithm
    % user can use global variables under the name of `user`
    % user can also use `user` to save the result of `Plan()` to use in `Control()`.
    %
    % Inputs:
    %   t: current time (scalar) [s]
    %   utc: current UTC (scalar, datetime)
    %   r: initial position of satellite (3 x 1) [m]
    %   v: initial velocity of satellite (3 x 1) [m/s]
    %   q: initial quaternion of satellite (4 x 1, q(4) is scalar part) [-]
    %   w: initial angular velocity of satellite (3 x 1) [rad/s]
    %   hw: initial angular momentum of rws (4 x 1) [Nms]
    %   targets: target list (n x 1, cell array). detail of i-th element is below (n x 1, cell array)
    %     targets{i}{TARGET_NAME_INDEX}: name of target (str)
    %     targets{i}{TARGET_CENTER_INDEX}: latitude and longitude of center of target (1 x 2) [deg, deg]
    %     targets{i}{TARGET_NE_INDEX}: latitude and longitude of north-east of target (1 x 2) [deg, deg]
    %     targets{i}{TARGET_NW_INDEX}: latitude and longitude of north-west of target (1 x 2) [deg, deg]
    %     targets{i}{TARGET_SW_INDEX}: latitude and longitude of south-west of target (1 x 2) [deg, deg]
    %     targets{i}{TARGET_SE_INDEX}: latitude and longitude of south-east of target (1 x 2) [deg, deg]
    %     targets{i}{TARGET_MAX_SCORE_INDEX}: maximum score (scalar)
    %

    global user
    global TARGET_NAME_INDEX TARGET_CENTER_INDEX TARGET_NE_INDEX TARGET_NW_INDEX TARGET_SW_INDEX TARGET_SE_INDEX TARGET_MAX_SCORE_INDEX

    % SAMPLE STRATEGY
    % 1. we want to make projected area of satellite body in the velocity direction as small as possible to reduce aerodynamic torque
    %       -> align body-Y with north-south (nearly velocity direction), body-X with west-east
    % 2. nadir view area (with attitude of strategy 1) is about 4km (north-south) x 6km (west-east) @ 300km alt
    %       -> we use only targets with west-east distance less than 6km because we can fill the target region with single sweep
    %       -> for example, the target region is 10km (north-south) x 5km (west-east), we can fill it 3 times of observation
    %       -> in this function, we prepare tracking points of each observation

    % nadir view area (north-south, m)
    fov_ns_m = (norm(r) - user.r_earth) * tan(deg2rad(user.fov_y));

    % use only targets west-east distance is 5km
    noshiro = targets{6};
    chiba = targets{28};
    user.use_targets = [
        SampleCreateTrackingPointsForNorthSouthSweep( ...
            noshiro{TARGET_NE_INDEX}, ...
            noshiro{TARGET_NW_INDEX}, ...
            noshiro{TARGET_SW_INDEX}, ...
            noshiro{TARGET_SE_INDEX}, ...
            fov_ns_m ...
        );
        SampleCreateTrackingPointsForNorthSouthSweep( ...
            chiba{TARGET_NE_INDEX}, ...
            chiba{TARGET_NW_INDEX}, ...
            chiba{TARGET_SW_INDEX}, ...
            chiba{TARGET_SE_INDEX}, ...
            fov_ns_m ...
        );
    ];
    user.current_target_index = 1;

    % prepare variables for controller
    user.mode = 1;
    user.rw_reverse_time = 0;
    user.e_prev = [0.0; 0.0; 0.0];  % previous error
    user.e_total = [0.0; 0.0; 0.0];  % integration of error
    user.kp = 0.24;  % P gain
    user.ki = 0.0;  % I gain
    user.kd = 3.9;  % D gain

end


function [points] = SampleCreateTrackingPointsForNorthSouthSweep(ne_latlon, nw_latlon, sw_latlon, se_latlon, fov_ns_m)
    global user

    % calc north-west distance
    required_distance_m = 2 * pi * user.r_earth * (ne_latlon(1) - se_latlon(1)) / 360;

    % how many times we need to observe to fill region
    count = ceil(required_distance_m / fov_ns_m);

    % fov_ns_m * count - required_distance_m -> ideal margin (no overlap)
    % to reduce risk of control error, we introduce overlap as half of ideal margin
    real_distance_m = required_distance_m + (fov_ns_m * count - required_distance_m) / 2;
    interval_m = real_distance_m / count;
    interval_lat = interval_m * 360 / (2 * pi * user.r_earth);
    northernmost_distance_m = real_distance_m / 2 - interval_m / 2;
    northernmost_point_lat = ne_latlon(1) - (ne_latlon(1) - se_latlon(1)) / 2 + northernmost_distance_m * 360 / (2 * pi * user.r_earth);
    points = cell(count, 1);
    lon = ne_latlon(2) - (ne_latlon(2) - nw_latlon(2)) / 2;
    for i=1:count
        points{i} = [northernmost_point_lat - (i - 1) * interval_lat, lon];
    end

end
