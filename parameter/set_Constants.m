function [] = set_Constants
global dth_ smt_ cpr_ pdp_ mu_ r_earth_
global mass_ II_ II_inv_ trq_max_ hw_max_ mtq_max_ conv_rw2body_ shapes_ Cd_
global los_ observation_interval_min_  observation_offnadir_max_ observation_los_speed_max_ observation_cover_min_ fov_corners_
global N_sv N_t N_r N_v N_q N_w N_hw
global FOV_CORNER_PXPY_INDEX FOV_CORNER_MXPY_INDEX FOV_CORNER_MXMY_INDEX FOV_CORNER_PXMY_INDEX
global PANEL_VERTICES_PX_INDEX PANEL_VERTICES_MX_INDEX
global PANEL_VERTICES_PY_INDEX PANEL_VERTICES_MY_INDEX
global PANEL_VERTICES_PZ_INDEX PANEL_VERTICES_MZ_INDEX
global user

%%% Simulation environment parameters
dth_ = 0.01;  %% step width of numerical integration [s]
smt_ = 300.0;  %% total simulation time [s]
cpr_ = 10;  %% 'Control' function called period [step] (if dth_=0.01 and cpr_=10, 'Control' function called every 0.1 (=0.01*10) seconds.)
pdp_ = 100;  %% geobasemap redraw period [step]
mu_ = 3.986004e+14;  %% gravity constant [m3/s2]
r_earth_ = 6378137.0;  % mean radius of the Earth [m]

%%% Satellite property
mass_ = 100.0;  % mass of satellite [kg]
II_ = [
    12.0 0.10 0.20;
    0.10 10.0 0.15;
    0.20 0.15 10.0;
];  % m.o.i of satellite [kgm2]
II_inv_ = inv(II_);
trq_max_ = 0.1;  % maximum allowable torque [Nm]
hw_max_ = 0.52;  % maximum momentum stored in the wheel [Nms]
mtq_max_ = 5.0;  % maximum allowable magnetic moment [Am2]
los_ = [0.0; 0.0; 1.0];  % telescope line of sight
FOV_x = deg2rad(1.2);  % telescope field of view x[rad]
FOV_y = deg2rad(0.8);  % telescope field of view y[rad]
fov_corners_ = {
    normalize([ tan(FOV_x / 2);  tan(FOV_y / 2); 1], "norm");  % PXPY
    normalize([-tan(FOV_x / 2);  tan(FOV_y / 2); 1], "norm");  % MXPY
    normalize([-tan(FOV_x / 2); -tan(FOV_y / 2); 1], "norm");  % MXMY
    normalize([ tan(FOV_x / 2); -tan(FOV_y / 2); 1], "norm");  % PXMY
}; % corner vector of FOV in body coordinate
FOV_CORNER_PXPY_INDEX = 1;
FOV_CORNER_MXPY_INDEX = 2;
FOV_CORNER_MXMY_INDEX = 3;
FOV_CORNER_PXMY_INDEX = 4;
observation_interval_min_ = 0.5;  % observation minimum interval [s]
observation_offnadir_max_ = deg2rad(30);  % maximum allowable offnadir angle when observation [rad]
observation_los_speed_max_ = 0.3 / (1 / 8000);  % maximum allowable speed of the point where los_ intersect the surface of the earth when observation [m/s]
observation_cover_min_ = 0.7;  % Ratio of the imaged area to the total area of the target region required to earn a score
conv_rw2body_ = [
    1.0 0.0 0.0 1/sqrt(2);
    0.0 1.0 0.0 1/sqrt(2);
    0.0 0.0 1.0 1/2;
];  % momentum conversion matrix wheel to body
l_body_x = 0.8;  % bus structure shape x [m]
l_body_y = 0.8;  % bus structure shape y [m]
l_body_z = 1.4;  % bus structure shape z [m]
l_panel_y = 3.2;  % PX SAP shape y [m]
l_panel_z = 1.4;  % PX SAP shape y [m]
offset_panel_y = l_body_y / 2;  % Y-direction offset of PX SAP
offset_cm = [0.05, 0.00, -0.05];  % center of mass offset from the geometric center of the main body [m]
px_shape_wo_offset_cm = [
    l_body_x / 2, -l_panel_y / 2 + offset_panel_y, -l_panel_z / 2;
    l_body_x / 2,  l_panel_y / 2 + offset_panel_y, -l_panel_z / 2;
    l_body_x / 2,  l_panel_y / 2 + offset_panel_y,  l_panel_z / 2;
    l_body_x / 2, -l_panel_y / 2 + offset_panel_y,  l_panel_z / 2;
];
mx_shape_wo_offset_cm = [
    -l_body_x / 2, -l_body_y / 2, -l_body_z / 2;
    -l_body_x / 2,  l_body_y / 2, -l_body_z / 2;
    -l_body_x / 2,  l_body_y / 2,  l_body_z / 2;
    -l_body_x / 2, -l_body_y / 2,  l_body_z / 2;
];
py_shape_wo_offset_cm = [
     l_body_x / 2,  l_body_y / 2, -l_body_z / 2;
    -l_body_x / 2,  l_body_y / 2, -l_body_z / 2;
    -l_body_x / 2,  l_body_y / 2,  l_body_z / 2;
     l_body_x / 2,  l_body_y / 2,  l_body_z / 2;
];
my_shape_wo_offset_cm = [
     l_body_x / 2, -l_body_y / 2, -l_body_z / 2;
    -l_body_x / 2, -l_body_y / 2, -l_body_z / 2;
    -l_body_x / 2, -l_body_y / 2,  l_body_z / 2;
     l_body_x / 2, -l_body_y / 2,  l_body_z / 2;
];
pz_shape_wo_offset_cm = [
     l_body_x / 2,  l_body_y / 2,  l_body_z / 2;
    -l_body_x / 2,  l_body_y / 2,  l_body_z / 2;
    -l_body_x / 2, -l_body_y / 2,  l_body_z / 2;
     l_body_x / 2, -l_body_y / 2,  l_body_z / 2;
];
mz_shape_wo_offset_cm = [
     l_body_x / 2,  l_body_y / 2, -l_body_z / 2;
    -l_body_x / 2,  l_body_y / 2, -l_body_z / 2;
    -l_body_x / 2, -l_body_y / 2, -l_body_z / 2;
     l_body_x / 2, -l_body_y / 2, -l_body_z / 2;
];
shapes_ = {
    (px_shape_wo_offset_cm - offset_cm)';
    (mx_shape_wo_offset_cm - offset_cm)';
    (py_shape_wo_offset_cm - offset_cm)';
    (my_shape_wo_offset_cm - offset_cm)';
    (pz_shape_wo_offset_cm - offset_cm)';
    (mz_shape_wo_offset_cm - offset_cm)';
};
PANEL_VERTICES_PX_INDEX = 1;
PANEL_VERTICES_MX_INDEX = 2;
PANEL_VERTICES_PY_INDEX = 3;
PANEL_VERTICES_MY_INDEX = 4;
PANEL_VERTICES_PZ_INDEX = 5;
PANEL_VERTICES_MZ_INDEX = 6;
Cd_ = 2.2;

%%% index to the save variables
N_sv = 18;  % number of save variables
N_t = 1;  % time
N_r = 2:4;  % position of the satellite
N_v = 5:7;  % velocity of the satellite
N_q = 8:11;  % quaternion of the satellite
N_w = 12:14;  % angular velocity of the satellite
N_hw = 15:18;  % momentum stored in the wheels

%%% For user controller
user.mass = mass_;
user.II = II_;
user.II_inv = II_inv_;
user.trq_max = trq_max_;
user.hw_max = hw_max_;
user.mtq_max = mtq_max_;
user.los = los_;
user.fov_x = rad2deg(FOV_x);
user.fov_y = rad2deg(FOV_y);
user.fov_corner_pxpy = fov_corners_{FOV_CORNER_PXPY_INDEX};
user.fov_corner_mxpy = fov_corners_{FOV_CORNER_MXPY_INDEX};
user.fov_corner_mxmy = fov_corners_{FOV_CORNER_MXMY_INDEX};
user.fov_corner_pxmy = fov_corners_{FOV_CORNER_PXMY_INDEX};
user.panel_vertices_px = shapes_{PANEL_VERTICES_PX_INDEX};
user.panel_vertices_mx = shapes_{PANEL_VERTICES_MX_INDEX};
user.panel_vertices_py = shapes_{PANEL_VERTICES_PY_INDEX};
user.panel_vertices_my = shapes_{PANEL_VERTICES_MY_INDEX};
user.panel_vertices_pz = shapes_{PANEL_VERTICES_PZ_INDEX};
user.panel_vertices_mz = shapes_{PANEL_VERTICES_MZ_INDEX};
user.Cd = Cd_;
user.observation_interval_min = observation_interval_min_;
user.observation_offnadir_max = observation_offnadir_max_;
user.observation_los_speed_max = observation_los_speed_max_;
user.observation_cover_min = observation_cover_min_;
user.conv_rw2body = conv_rw2body_;
user.dt_control = dth_ * cpr_;
user.r_earth = r_earth_;
user.mu = mu_;
