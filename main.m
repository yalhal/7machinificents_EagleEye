
clear all;
close all;

addpath("dynamics");
addpath("environment");
addpath("graph");
addpath("observation");
addpath("parameter");
addpath("user");
addpath("utility");

warning off MATLAB:polyshape:repairedBySimplify
warning off MATLAB:polyshape:boolOperationFailed

global dth_ smt_ cpr_ pdp_
global N_t N_r N_hw N_sv N_v N_w N_q
global t_ t_prev_ utc_ utc_prev_ x_ x_prev_
global targets_

set_Constants;
set_Targets;
set_InitialValues;
t_ = 0;
t_prev_ = 0;
step = 0;
save_count = 1;

Plan(t_, utc_, x_(N_r), x_(N_v), x_(N_q), x_(N_w), x_(N_hw), targets_);

ts = zeros(smt_ / (dth_ * cpr_) + 1, 1); % 追加
xs = zeros(smt_ / (dth_ * cpr_) + 1, N_sv);

while (t_ < smt_)
    % environment update
    [lat, lon, alt] = ecef2lla(eci2ecef(x_(N_r), utc_));
    mag_field_ecef = MagneticField(lat, lon, alt);
    air_rho = AirDensity(alt);

    % user control function
    if rem(step, cpr_) == 0
        % [T_rw, M_mtq, is_observe] = ControlOriginal(t_, utc_, x_(N_r), x_(N_v), x_(N_q), x_(N_w), x_(N_hw), q2dcm(x_(N_q)) * ecef2eci(mag_field_ecef, utc_));
        [T_rw, M_mtq, is_observe] = Control(t_, utc_, x_(N_r), x_(N_v), x_(N_q), x_(N_w), x_(N_hw), q2dcm(x_(N_q)) * ecef2eci(mag_field_ecef, utc_));
    end

    if rem(step, pdp_) == 0
        UpdateGeobasemap(lat, lon, 0);
        UpdateStateGraph(T_rw, M_mtq);
        if(save_count>1) %追加
            CreateStateGraphUser(ts(1:save_count, :), xs(1:save_count, :), save_count); %追加
            CreateBodyFigUser(utc_, x_(N_r), x_(N_v), x_(N_q)); % 追加
        end %追加
        drawnow;
    end

    % check limitation
    T_rw = CheckRwConstraint(T_rw);
    M_mtq = CheckMtqConstraint(M_mtq);

    % save variable
    if rem(step, cpr_) == 0
        xs(save_count, :) = x_;
        save_count = save_count + 1;
        ts(save_count, 1) = t_; % 追加
    end

    % observation
    if is_observe
        [index, ~, pxpy, mxpy, mxmy, pxmy, ~, ~] = IsValidObservation();
        if index ~= 0
            Observe(index, [pxpy; mxpy; mxmy; pxmy]);
            UpdateObservedAreaPlot(index);
            UpdateGeobasemap(lat, lon, index);
            drawnow;
        end
    end

    % propagate
    StateUpdate(air_rho, mag_field_ecef, T_rw, M_mtq);

    % save variable
    x_prev_ = x_;
    t_prev_ = t_;
    utc_prev_ = utc_;

    % time increment
    t_ = t_ + dth_;
    x_(N_t) = t_;
    utc_ = utc_ + seconds(dth_);
    step = step + 1;

end


writematrix(xs, "result.csv")

