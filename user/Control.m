function [T_rw, M_mtq, is_observe] = Control(t, utc, r, v, q, w, hw, mag)
    % user-defined control algorithm
    % user can use global variables under the name of `user`
    %
    % Inputs:
    %   t: current time (scalar) [s]
    %   utc: current UTC (scalar, datetime)
    %   r: position of satellite (3 x 1) [m]
    %   v: velocity of satellite (3 x 1) [m/s]
    %   q: quaternion of satellite (4 x 1, q(4) is scalar part) [-]
    %   w: angular velocity of satellite (3 x 1) [rad/s]
    %   hw: angular momentum of rws (4 x 1) [Nms]
    %   mag: magnetic field vector in body coodinate (3 x 1) [T]
    %
    % Returns:
    %   T_rw: control torque of rws (4 x 1, `-user.conv_rw2body_ * T_rw` represents control torque of satellite) [Nm]
    %   M_mtq: output magnetic moment of mtqs (3 x 1) [Am2]
    %   is_observe: control flag to observe or not in current time.
    %               Observation is executed when this variable is true and IsValidObservation() returns valid values.

    global user

    % at first, change attitude from PX-Sun to PX-Velocity angular moment (Z rotation)
    % in this step, control torque calculated roughly by feed-forward
    if user.mode == 1
        if user.rw_reverse_time == 0
            cross_vector = cross([1.0; 0.0; 0.0], v);
            torque_dir_y = sign(cross_vector(2));
            angle = acos(v(1));
            % time that rw(z) saturate with continuous maximum torque
            t_saturate = user.hw_max / user.trq_max;
            % total change angle and angular_velocity until saturation
            angle_saturate = 0.5 * user.trq_max * user.II_inv(3, 3) * t_saturate * t_saturate;
            w_saturate = user.trq_max * user.II_inv(3, 3) * t_saturate;
            if angle / 2 < angle_saturate
                user.rw_reverse_time = sqrt((angle / 2) / (0.5 * user.trq_max * user.II_inv(3, 3)));
                user.mode_change_time = 2 * user.rw_reverse_time;
            else
                user.rw_reverse_time = t_saturate + (angle - 2 * angle_saturate) / w_saturate;
                user.mode_change_time = user.rw_reverse_time + t_saturate;
            end
            user.torque_dir_y = torque_dir_y;
            T_rw = [0; -user.torque_dir_y * user.trq_max; 0; 0];
            M_mtq = [0; 0; 0];
            is_observe = false;
            return
        elseif t <= user.rw_reverse_time
            T_rw = [0; -user.torque_dir_y * user.trq_max; 0; 0];
            M_mtq = [0; 0; 0];
            is_observe = false;
            return
        elseif user.rw_reverse_time <= t && t < user.mode_change_time
            T_rw = [0; user.torque_dir_y * user.trq_max; 0; 0];
            M_mtq = [0; 0; 0];
            is_observe = false;
            return
        elseif user.mode_change_time <= t
            user.mode = 2;
            user.rw_reverse_time = 0;
            user.mode_change_time = 0;
        end
    end

    % which is current target
    if user.current_target_index > length(user.use_targets)
        % all target used
        T_rw = [0; 0; 0; 0];
        M_mtq = [0; 0; 0];
        is_observe = false;
        return
    elseif user.current_target_index < length(user.use_targets)
        % check distance between satellite and target
        % if next target is closer than current target, change target
        ct = user.use_targets{user.current_target_index};
        current_target_eci = ecef2eci(lla2ecef(ct(1), ct(2), 0), utc);
        nt = user.use_targets{user.current_target_index + 1};
        next_target_eci = ecef2eci(lla2ecef(nt(1), nt(2), 0), utc);
        current_distance = norm(r - current_target_eci);
        next_distance = norm(r - next_target_eci);
        if current_distance > next_distance
            user.current_target_index = user.current_target_index + 1;
        end
    end

    % calculate target attitude
    % LOS(=PZ) -> satellite to target, PX -> east (cross of r and r to south)
    target_latlon = user.use_targets{user.current_target_index};
    target_eci = ecef2eci(lla2ecef(target_latlon(1), target_latlon(2), 0), utc);
    alignment = normalize(target_eci - r, "norm");
    % constraint = normalize(cross(r, [0.0; 0.0; -user.r_earth] - r), "norm");
    constraint = normalize(cross(alignment, v), "norm");
    % orthogonalization
    third = normalize(cross(alignment, constraint), "norm");
    % constraint = normalize(cross(third, alignment), "norm");
    target_dcm = [constraint';third';alignment'];
    % calculate error as vector part of quaternion error
    current_dcm = q2dcm(q);
    error_dcm = target_dcm * current_dcm';
    error_q = dcm2q(error_dcm);
    e = error_q(1:3);
    error_angle = rad2deg(acos(dot(user.los, current_dcm * alignment)));

    % if control error is small, exec observation and change target for next step
    if error_angle < user.fov_x / 4
        [index, ~, ~, ~, ~, ~, ~, ~] = IsValidObservation();
        if index ~= 0
            is_observe = true;
            user.current_target_index = user.current_target_index + 1;
            user.e_total = 0.0;
        else
            is_observe = false;
        end
    else
        is_observe = false;
    end

    % if target is close, change gain
    nadir_body = current_dcm * normalize(-r, "norm");
    target_body = current_dcm * alignment;
    offnadir = rad2deg(acos(dot(nadir_body, target_body)));
    if offnadir < 50
        user.kp = 1.4;
        user.kd = 1.9;
        user.ki = 0.03;
    else
        user.kp = 0.24;
        user.kd = 3.9;
        user.ki = 0.00;
        user.e_total = 0;
    end

    % PID controller
    dt = user.dt_control;
    e_i = (user.e_prev + e) * dt / 2;
    user.e_total = user.e_total + e_i;
    u = user.kp * e + user.ki * user.e_total + user.kd * (e - user.e_prev) / dt;
    user.e_prev = e;

    % %% torque distribution: skew not used
    % T_rw = [-u; 0.0];

    % torque distribution: skew not used
    A = [1, 0, 0, cos(pi/4)*cos(pi/4);
         0, 1, 0, sin(pi/4)*cos(pi/4);
         0, 0, 1, sin(pi/4)];
    T_rw = pinv(A)*(-u);
    % T_rw = T_rw.*0;
    
    % MTQ not used
    M_mtq = [0; 0; 0];

    % %% MTQ cross product %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % hw_xyz = hw(1:3);  % xyz方向のRWの角運動量を取り出す
    % hw_n = hw_xyz/norm(hw_xyz);  % hwの正規化
    % B_n = mag/norm(mag);  % 地磁気ベクトルの正規化
    % if abs(dot(hw_n, B_n)) < 0.5  % θが60度以上ならば磁気トルカを稼働
    %     dir = cross(hw_n, B_n);  % 磁気モーメントの方向を定める
    %     dir_unit = dir/norm(dir);  % 方向ベクトルの正規化
    %     k_max = min(5.0 ./ abs(dir_unit));  % 方向ベクトルの３成分のうち最も大きい成分をmtq_maxに合わせる
    %     M_mtq = k_max*dir_unit;
    % else
    %     M_mtq = [0; 0; 0];
    % end
    % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


    % % MTQ MAX %%%%%%%%%%%%%%
    % M_mtq = [5.0; 5.0; 5.0];
    % %%%%%%%%%%%%%%%%%%%%%%%%

end
