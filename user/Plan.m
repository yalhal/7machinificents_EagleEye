function [] = Plan(t, utc, r, v, q, w, hw, targets)
    % ユーザー定義の計画アルゴリズム
    % グローバル変数 `user` を使用可能
    % `Plan()` の結果を `user` に保存し、`Control()` で利用可能
    %
    % 入力:
    %   t: 現在時刻（スカラー）[s]
    %   utc: 現在のUTC（スカラー, datetime）
    %   r: 衛星の初期位置（3 x 1）[m]
    %   v: 衛星の初期速度（3 x 1）[m/s]
    %   q: 衛星の初期クォータニオン（4 x 1, q(4)がスカラー部）[-]
    %   w: 衛星の初期角速度（3 x 1）[rad/s]
    %   hw: RWの初期角運動量（4 x 1）[Nms]
    %   targets: ターゲットリスト（n x 1, セル配列）。各要素の詳細は以下（n x 1, セル配列）
    %     targets{i}{TARGET_NAME_INDEX}: ターゲット名（文字列）
    %     targets{i}{TARGET_CENTER_INDEX}: ターゲット中心の緯度経度（1 x 2）[deg, deg]
    %     targets{i}{TARGET_NE_INDEX}: ターゲット北東端の緯度経度（1 x 2）[deg, deg]
    %     targets{i}{TARGET_NW_INDEX}: ターゲット北西端の緯度経度（1 x 2）[deg, deg]
    %     targets{i}{TARGET_SW_INDEX}: ターゲット南西端の緯度経度（1 x 2）[deg, deg]
    %     targets{i}{TARGET_SE_INDEX}: ターゲット南東端の緯度経度（1 x 2）[deg, deg]
    %     targets{i}{TARGET_MAX_SCORE_INDEX}: 最大スコア（スカラー）
    %

    global user
    global TARGET_NAME_INDEX TARGET_CENTER_INDEX TARGET_NE_INDEX TARGET_NW_INDEX TARGET_SW_INDEX TARGET_SE_INDEX TARGET_MAX_SCORE_INDEX

    % サンプル戦略
    % 1. 衛星本体の進行方向（速度方向）に投影される面積を最小化し、空気力学的トルクを低減したい
    %       -> ボディY軸を南北（ほぼ速度方向）、ボディX軸を東西に合わせる
    % 2. この姿勢でのナディア観測領域は高度300kmで約4km（南北）×6km（東西）
    %       -> 東西幅が6km未満のターゲットのみ、1回の観測で全域をカバーできるので使用
    %       -> 例えば、ターゲット領域が10km（南北）×5km（東西）なら、3回の観測で全域をカバーできる
    %       -> この関数では各観測のトラッキングポイントを準備する

    % ナディア観測幅（南北方向, m）
    % 視野fov_yと現在の高度norm(r)から、衛星の真下に投影される南北方向の観測幅（[m]）を計算
    fov_ns_m = (norm(r) - user.r_earth) * tan(deg2rad(user.fov_y));

    % 7/16川﨑追記：横方向の掃引のため
    fov_ew_m = (norm(r) - user.r_earth) * tan(deg2rad(user.fov_x));

    % 東西幅が5kmのターゲットのみ使用
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

    % コントローラ用変数の準備
    user.mode = 1;
    user.rw_reverse_time = 0;
    user.e_prev = [0.0; 0.0; 0.0];  % 前回の誤差
    user.e_total = [0.0; 0.0; 0.0];  % 誤差の積分値
    user.kp = 0.24;  % Pゲイン
    user.ki = 0.0;   % Iゲイン
    user.kd = 3.9;   % Dゲイン

end

% 7/16川﨑作成
function [points] = SampleCreateTrackingPointsForNorthSouthSweep(ne_latlon, nw_latlon, sw_latlon, se_latlon, fov_ns_m)
    % ターゲット領域をFOVで分割して南北方向の観測点を作る関数
    %   入力：
    %   ne_latlon: ターゲット領域の北東端の緯度経度（1 x 2）[deg, deg]
    %   nw_latlon: ターゲット領域の北西端の緯度経度（1 x 2）[deg, deg]
    %   sw_latlon: ターゲット領域の南西端の緯度経度（1 x 2）[deg, deg]
    %   se_latlon: ターゲット領域の南東端の緯度経度（1 x 2）[deg, deg]
    %   fov_ns_m: 観測可能な南北方向の長さ（視野）[m]
    %   出力：
    %   points: 各観測点の中心緯度経度のリスト（cell配列）
    global user

    % 南北方向の距離[m]を計算
    required_distance_m = 2 * pi * user.r_earth * (ne_latlon(1) - se_latlon(1)) / 360;

    % 領域をカバーするのに必要な観測回数
    count = ceil(required_distance_m / fov_ns_m);

    % fov_ns_m * count - required_distance_m -> 理想的なマージン（重複なし）
    % 制御誤差リスク低減のため、理想マージンの半分を重複として導入
    % 川﨑コメント：例えばターゲット領域の南北が7.4kmでfov_ns_mが3kmとする．観測回数は切り上げて3回となるが，3*3-7.4=1.6kmの余りが生まれる．このうち半分を重複としてのりしろとして使用（残りの半分は観測範囲の外）
    real_distance_m = required_distance_m + (fov_ns_m * count - required_distance_m) / 2;
    interval_m = real_distance_m / count;
    interval_lat = interval_m * 360 / (2 * pi * user.r_earth);
    northernmost_distance_m = real_distance_m / 2 - interval_m / 2;
    northernmost_point_lat = ne_latlon(1) - (ne_latlon(1) - se_latlon(1)) / 2 + northernmost_distance_m * 360 / (2 * pi * user.r_earth);
    points = cell(count, 1);
    lon = ne_latlon(2) - (ne_latlon(2) - nw_latlon(2)) / 2;
    % 観測点（緯度経度）のリストを作成
    for i=1:count
        points{i} = [northernmost_point_lat - (i - 1) * interval_lat, lon];
    end

end

% 7/16川﨑作成
% ターゲット領域をFOVで分割して東西方向の観測点を作る関数
% 入力：
%   ne_latlon: ターゲット領域の北東端の緯度経度（1 x 2）[deg, deg]
%   nw_latlon: ターゲット領域の北西端の緯度経度（1 x 2）[deg, deg]
%   sw_latlon: ターゲット領域の南西端の緯度経度（1 x 2）[deg, deg]
%   se_latlon: ターゲット領域の南東端の緯度経度（1 x 2）[deg, deg]
%   fov_ew_m: 観測可能な東西方向の長さ（視野）[m]
% 出力：
%   points: 各観測点の中心緯度経度のリスト（cell配列）

function [points] = SampleCreateTrackingPointsForEastWestSweep(ne_latlon, nw_latlon, sw_latlon, se_latlon, fov_ew_m)
    global user

    % 中心緯度を計算（経度→距離変換のために必要）
    lat_avg = (nw_latlon(1) + sw_latlon(1)) / 2;

    % 経度1度あたりの距離[m]（緯度によって変化）
    meters_per_deg_lon = 2 * pi * user.r_earth * cosd(lat_avg) / 360;

    % 東西方向の距離[m]を計算
    required_distance_m = abs(nw_latlon(2) - ne_latlon(2)) * meters_per_deg_lon;

    % 領域をカバーするのに必要な観測回数
    count = ceil(required_distance_m / fov_ew_m);

    % 重複を含めた実際の距離
    real_distance_m = required_distance_m + (fov_ew_m * count - required_distance_m) / 2;
    interval_m = real_distance_m / count;
    interval_lon = interval_m / meters_per_deg_lon;

    % 最西端の観測点の経度
    westernmost_distance_m = real_distance_m / 2 - interval_m / 2;
    westernmost_point_lon = nw_latlon(2) + (ne_latlon(2) - nw_latlon(2)) / 2 - westernmost_distance_m / meters_per_deg_lon;

    % 中心緯度は固定（東西掃引なので緯度は同じ）
    lat = nw_latlon(1) - (nw_latlon(1) - sw_latlon(1)) / 2;

    % 観測点（緯度経度）のリストを作成
    points = cell(count, 1);
    for i = 1:count
        points{i} = [lat, westernmost_point_lon + (i - 1) * interval_lon];
    end
end

% 7/16川﨑作成
function [points] = CreateTrackingPointsAutoSweep(ne_latlon, nw_latlon, sw_latlon, se_latlon, fov_m)
    % ターゲット領域の縦横比に応じて南北または東西方向に観測点を生成する関数
    % fov_m: 視野幅（南北掃引なら南北方向[m]、東西掃引なら東西方向[m]）
    % 出力 points: cell 配列 (1 x 掃引数)、各要素は [lat, lon]

    global user

    % --- 1. 緯度・経度の差をもとに領域の寸法（m）を計算 ---
    lat_center = (ne_latlon(1) + se_latlon(1)) / 2;
    deg2m_lat = 2 * pi * user.r_earth / 360;
    deg2m_lon = 2 * pi * user.r_earth * cosd(lat_center) / 360;

    ns_length = abs(ne_latlon(1) - se_latlon(1)) * deg2m_lat;  % 南北方向の長さ
    ew_length = abs(ne_latlon(2) - nw_latlon(2)) * deg2m_lon;  % 東西方向の長さ

    % --- 2. 長い方を掃引方向に選ぶ ---
    if ns_length >= ew_length
        % 南北掃引
        points = SampleCreateTrackingPointsForNorthSouthSweep( ...
            ne_latlon, nw_latlon, sw_latlon, se_latlon, fov_m ...
        );
    else
        % 東西掃引
        points = SampleCreateTrackingPointsForEastWestSweep( ...
            ne_latlon, nw_latlon, sw_latlon, se_latlon, fov_m ...
        );
    end
end
