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


    % Stable Attitude
    if user.mode == 1

        % Target DCM
        % X -> Orbit Vec. / Y -> Velocity Vec.
        target_X_vec = normalize(cross(r, v), "norm");
        target_Y_vec = normalize(v, "norm");
        target_Z_vec = normalize(cross(target_X_vec, target_Y_vec), "norm");
        target_dcm   = [target_X_vec'; target_Y_vec'; target_Z_vec'];
        
        % Error DCM
        current_dcm  = q2dcm(q);
        error_dcm    = target_dcm * current_dcm';
        error_q      = dcm2q(error_dcm);

        % Error Euler Angle / Angular Vel.
        error_attitude = 2.0 .* [error_q(1); error_q(2); error_q(3)]
        error_angvel   = - w;

        % Integration of Error
        if ~exist('user.error_integed', 'var')
            user.error_integed = 0.0;
        end
        error_traped       = (user.e_prev + error_attitude) * user.dt_control / 2;
        user.error_integed = user.error_integed + error_traped;

        % PID Control
        user.kp = 1.4;
        user.kd = 1.8;
        user.ki = 0.00;
        input = - user.kp .* error_attitude - user.ki * user.error_integed - user.kd .* error_angvel;

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
        
        % Judge Convergence
        if (max(abs(error_attitude)) < deg2rad(10))
            user.mode = 2;
            user.error_integed = 0;
        end
        
        return
    end


    % Observation
    if user.mode == 2
        % Target Selection
        if user.current_target_index < length(user.use_targets)
            % Distance to Current Target
            curr_target_lla = user.use_targets{user.current_target_index};
            curr_target_eci = ecef2eci(lla2ecef(curr_target_lla(1), curr_target_lla(2), 0), utc);
            curr_target_dis = norm(r - curr_target_eci);
            % Distance to Next Target
            next_target_lla = user.use_targets{user.current_target_index + 1};
            next_target_eci = ecef2eci(lla2ecef(next_target_lla(1), next_target_lla(2), 0), utc);
            next_target_dis = norm(r - next_target_eci);
            % Compare
            if curr_target_dis > next_target_dis
                user.current_target_index = user.current_target_index + 1;
            end
        elseif user.current_target_index > length(user.use_targets)
            user.mode = 3;          % all target Passed
        end
    
        % Target DCM
        % Z -> Target, X -> Orbit Vec. (as possible)
        target_latlon = user.use_targets{user.current_target_index};
        target_eci    = ecef2eci(lla2ecef(target_latlon(1), target_latlon(2), 0), utc);
        target_Z_vec  = normalize(target_eci - r, "norm");
        target_Y_vec  = normalize(cross(target_Z_vec, normalize(cross(r, v), "norm")), "norm");
        target_X_vec = normalize(cross(target_Y_vec, target_Z_vec), "norm");
        target_dcm   = [target_X_vec'; target_Y_vec'; target_Z_vec'];
        
        % Error DCM
        current_dcm  = q2dcm(q);
        error_dcm    = target_dcm * current_dcm';
        error_q      = dcm2q(error_dcm);

        % Error Euler Angle / Angular Vel.
        error_attitude = 2.0 .* [error_q(1); error_q(2); error_q(3)]
        error_angvel   = - w;

        % Error CAM angle
        error_angle = rad2deg(acos(dot(user.los, current_dcm*target_Z_vec)));

        % Judge of Observation
        if (error_attitude(2) < user.fov_x / 4 && error_attitude(1) < deg2rad(5))
            [index, ~, ~, ~, ~, ~, ~, ~] = IsValidObservation();
            if index ~= 0
                is_observe = true;
                user.current_target_index = user.current_target_index + 1;
                user.error_integed = 0.0;
            else
                is_observe = false;
            end
        else
            is_observe = false;
        end
    
        % if target is close, change gain
        nadir_body  = current_dcm * normalize(-r, "norm");
        target_body = current_dcm * target_Z_vec;
        offnadir = rad2deg(acos(dot(nadir_body, target_body)));
        if (offnadir < 50)
            user.kp = 1.4;
            user.kd = 1.9;
            user.ki = 0.03;
        else
            user.kp = 0.24;
            user.kd = 3.9;
            user.ki = 0.00;
            user.error_integed = 0;
        end

        % Integration of Error
        error_traped       = (user.e_prev + error_attitude) * user.dt_control / 2;
        user.error_integed = user.error_integed + error_traped;

        % PID controller
        user.kp = 1.4;
        user.kd = 1.8;
        user.ki = 0.00;
        input = - user.kp .* error_attitude - user.ki * user.error_integed - user.kd .* error_angvel;

        % Distribution Law
        u     = input;
        if max(u) > user.trq_max
            T_rw = pinv(A)*u;
        else
            T_rw = [u; 0.0];
        end

        % B-dot
        if (offnadir < 50)
            if ~exist('user.mag_prev', 'var')
                user.mag_prev = mag;
            end
            B_dot = (mag - user.mag_prev) / user.dt_control;
            user.mag_prev = mag;
            user.kb       = 1e4;

            M_mtq = -user.kb .* B_dot .*0;
        else
            M_mtq = [0; 0; 0];
        end
        return
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


