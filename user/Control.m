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

    % Properties
    A = [1, 0, 0, cos(pi/4)*cos(pi/4);
         0, 1, 0, sin(pi/4)*cos(pi/4);
         0, 0, 1, sin(pi/4)];

    % X -> Orbit Vec. / Y -> Velocity Vec.
    if user.mode == 1
        % Target DCM
        target_X_vec = normalize(cross(r, v), "norm");
        target_Y_vec = normalize(v, "norm");
        target_Z_vec = normalize(cross(target_X_vec, target_Y_vec), "norm");
        target_dcm   = [target_X_vec'; target_Y_vec'; target_Z_vec']';
        
        % Error DCM
        current_dcm  = q2dcm(q);
        error_dcm    = target_dcm * current_dcm';
        error_q      = dcm2q(error_dcm);

        % Error Euler Angle / Angular Vel.
        error_attitude = 2.0 .* [error_q(1); error_q(2); error_q(3)]
        error_angvel   = - w;

        % PID Control
        user.kp = 1.4;
        user.kd = 1.8;
        input = - user.kp .* error_attitude - user.kd .* error_angvel;

        % LQR (not used)
        %{
        A = [0, 1; 0, 0];
        B = [0; 1/12];
        Q = [1, 0; 0, 1];
        R = 0.1;
        [K, ~] = ControlLQR(A, B, Q, R)
        input_x = -K * [error_attitude(1); w(1)];

        B = [0; 1/10];
        [K, ~] = ControlLQR(A, B, Q, R);
        input_y = -K * [error_attitude(2); w(2)];
        input_z = -K * [error_attitude(3); w(3)];

        %input   = [input_x; input_y; input_z];
        % input   = [0; 0; input_z];
        %}

        % Distribution Law
        u     = input;
        if max(u) > user.trq_max
            T_rw = pinv(A)*u;
        else
            T_rw = [u; 0.0];
        end

        % Cross Product
        if (max(abs(error_attitude)) < deg2rad(15))
            limit_cross = cos(deg2rad(70));
            hw_vec  = A*hw;
            inner_vec = abs(dot(hw_vec./norm(hw_vec), mag./norm(mag)));
            display(rad2deg(acos(inner_vec)))
            if (inner_vec < limit_cross)
                user.k_mtq = 100.0;
                M_mtq = -user.k_mtq .* cross(hw_vec, mag);
                M_mtq = M_mtq .* 0;
            else
                M_mtq = [0; 0; 0];
            end
        else
            M_mtq = [0; 0; 0];
        end

        % Observation
        is_observe = false;
        
        return
    end


    % Observation
    if user.mode == 2
        % Target selection
        if user.current_target_index > length(user.use_targets)
            % all target Passed
            user.mode = 3;
            T_rw = [0; 0; 0; 0];
            M_mtq = [0; 0; 0];
            is_observe = false;
            return
        elseif user.current_target_index < length(user.use_targets)
            % Check distance between satellite and target
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
    
        % Target DCM
        % Z -> Target, X -> east (cross of r and r to south)
        target_latlon = user.use_targets{user.current_target_index};
        target_eci    = ecef2eci(lla2ecef(target_latlon(1), target_latlon(2), 0), utc);
        target_Z_vec  = normalize(target_eci - r, "norm");


        target_dcm   = [target_X_vec'; target_Y_vec'; target_Z_vec']';

        % constraint = normalize(cross(r, [0.0; 0.0; -user.r_earth] - r), "norm");
        constraint = normalize(cross(target_Z_vec, v), "norm");
        % orthogonalization
        third = normalize(cross(target_Z_vec, constraint), "norm");
        % constraint = normalize(cross(third, alignment), "norm");
        target_dcm = [constraint';third';target_Z_vec'];
        % calculate error as vector part of quaternion error
        current_dcm = q2dcm(q);
        error_dcm = target_dcm * current_dcm';
        error_q = dcm2q(error_dcm);
        e = error_q(1:3);
        error_angle = rad2deg(acos(dot(user.los, current_dcm * target_Z_vec)));
    
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
        target_body = current_dcm * target_Z_vec;
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
    
        % torque distribution: skew used
        A = [1, 0, 0, cos(pi/4)*cos(pi/4);
             0, 1, 0, sin(pi/4)*cos(pi/4);
             0, 0, 1, sin(pi/4)];
        T_rw = pinv(A)*(-u);
        % T_rw = T_rw.*0;
        
        % MTQ not used
        M_mtq = [0; 0; 0];
    end

       
    % Free-Flight
    if user.mode == 3
        T_rw = [0; 0; 0; 0];
        M_mtq = [0; 0; 0];
        is_observe = false;
        return
    end

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


function q_c = quaternion_multiply(q_a, q_b) 
    xa = q_a(1);
    ya = q_a(2);
    za = q_a(3);
    wa = q_a(4);
    
    xb = q_b(1);
    yb = q_b(2);
    zb = q_b(3);
    wb = q_b(4);

    wc = wa*wb - xa*xb - ya*yb - za*zb;
    xc = wa*xb + xa*wb + ya*zb - za*yb;
    yc = wa*yb - xa*zb + ya*wb + za*xb;
    zc = wa*zb + xa*yb - ya*xb + za*wb;
    
    q_c = [xc, yc, zc, wc];
end


function q_conj = quaternion_conjection(q)
    q_conj = [-q(1), -q(2), -q(3), q(4)];
end


