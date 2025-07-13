function [] = CreateBodyFigUser(q, utc, r_ecef)
    % --- 1. 基本となる直方体の寸法を定義（スケール調整） ---
    scale = 300000; % 衛星のスケール（実際のサイズを1000倍に拡大）
    width1  = 0.8 * scale; % x_body軸方向の幅
    length1 = 0.8 * scale; % y_body軸方向の長さ
    height1 = 1.4 * scale; % z_body軸方向の高さ

    % --- 1.5. クォータニオンから回転行列への変換 ---
    % ボディ座標系から慣性座標系（ECI）への変換
    R_q = q2dcm(q);

    % 慣性座標系（ECI）から地球固定座標系（ECEF）への変換
    timehack = [year(utc), month(utc), day(utc), hour(utc), minute(utc), second(utc)];
    gast = siderealtime(timehack);
    R_eci2ecef = CoordinateRotation("z", gast);

    % 合成回転: ボディ座標系 → 地球固定座標系（ECEF）
    R_total = R_eci2ecef * R_q;
    
    % --- 2. 直方体の頂点(Vertices)と面(Faces)をボディ座標系で定義 ---
    vertices1 = [
        -width1/2, -length1/2, -height1/2; 
         width1/2, -length1/2, -height1/2;
         width1/2,  length1/2, -height1/2;
        -width1/2,  length1/2, -height1/2;
        -width1/2, -length1/2,  height1/2;
         width1/2, -length1/2,  height1/2;
         width1/2,  length1/2,  height1/2;
        -width1/2,  length1/2,  height1/2 
    ];
    faces1 = [
        1, 2, 3, 4; % 底面
        5, 6, 7, 8; % 上面
        1, 2, 6, 5; % 前面
        4, 3, 7, 8; % 後面
        1, 4, 8, 5; % 左面
        2, 3, 7, 6  % 右面
    ];
    
    % --- 3. 上部に載せる長方形（ソーラーパネル）の寸法と位置をボディ座標系で定義 ---
    length2 = 3.4 * scale; % y_body軸方向の長さ
    height2  = 1.4 * scale; % z_body軸方向の高さ
    % ★直方体の上面の高さに、さらに0.05だけオフセットを追加
    x_position = (width1 / 2) + 0.05 * scale; 
    
    % --- 4. 長方形の頂点と面をボディ座標系で定義 ---
    vertices2 = [
        x_position, -length2/2, -height2/2;
        x_position, -length2/2,  height2/2;
        x_position,  length2/2,  height2/2;
        x_position,  length2/2, -height2/2
    ];
    faces2 = [1, 2, 3, 4]; % 面は1つだけ

    % --- 4.5. -Y面に貼り付ける円の定義をボディ座標系で追加 ---
    radius_c = 0.6 / 2 * scale; % 直径1なので半径は0.5
    num_points = 50; % 円を表現するための頂点数
    theta_c = linspace(0, 2*pi, num_points); % 0から2πまでの角度ベクトル
    % 円の頂点座標を計算 (+Z平面上に作成)
    x_coords_c = radius_c * cos(theta_c);
    y_coords_c = radius_c * sin(theta_c);
    % Z座標を直方体の+Z面から+0.05だけずらした位置に固定
    z_coord_c = width1/2 + 0.05 * scale; 
    z_coords_c = ones(1, num_points) * z_coord_c;
    % 頂点行列を作成
    vertices3 = [x_coords_c', y_coords_c', z_coords_c'];
    % 円の面を定義 (1からNまでの頂点を全て結ぶ)
    faces3 = 1:num_points;
    
    % --- 4.9. 回転と位置の適用 ---
    % ボディ座標系からECEF座標系への変換
    vertices1_rotated = vertices1*R_total';
    vertices2_rotated = vertices2*R_total';
    vertices3_rotated = vertices3*R_total';
    
    % ECEF座標系での衛星位置を加算
    vertices1_ecef = vertices1_rotated + r_ecef';
    vertices2_ecef = vertices2_rotated + r_ecef';
    vertices3_ecef = vertices3_rotated + r_ecef';
    
    % ECEF座標をLLA座標に変換
    % 衛星の位置をLLA座標に変換
    [lat_sat, lon_sat, alt_sat] = ecef2lla(r_ecef);
    
    % 各頂点をLLA座標に変換
    vertices1_lla = zeros(size(vertices1_ecef));
    vertices2_lla = zeros(size(vertices2_ecef));
    vertices3_lla = zeros(size(vertices3_ecef));
    
    for i = 1:size(vertices1_ecef, 1)
        [lat, lon, alt] = ecef2lla(vertices1_ecef(i, :));
        vertices1_lla(i, :) = [lon, lat, alt/1000]; % 経度, 緯度, 高度(km)
    end
    
    for i = 1:size(vertices2_ecef, 1)
        [lat, lon, alt] = ecef2lla(vertices2_ecef(i, :));
        vertices2_lla(i, :) = [lon, lat, alt/1000]; % 経度, 緯度, 高度(km)
    end
    
    for i = 1:size(vertices3_ecef, 1)
        [lat, lon, alt] = ecef2lla(vertices3_ecef(i, :));
        vertices3_lla(i, :) = [lon, lat, alt/1000]; % 経度, 緯度, 高度(km)
    end

    % --- 5. プロットの実行 ---
    figure(22); % 新しい図ウィンドウを作成
    clf;
    
    % patch関数で直方体を描画
    patch('Vertices', vertices1_lla, 'Faces', faces1, ...
          'FaceColor', [1.0, 0.65, 0.0], ... % オレンジ色のRGB値
          'FaceAlpha', 0.8, ...              
          'EdgeColor', 'black');             
    
    hold on; % 現在のグラフを保持して、上書きせずに次のプロットを追加
    
    % patch関数で上部の長方形（ソーラーパネル）を描画
    patch('Vertices', vertices2_lla, 'Faces', faces2, ...
          'FaceColor', [0.10, 0.10, 0.44], ... % ★色を青色のRGB値に変更
          'FaceAlpha', 0.75, ...              
          'EdgeColor', 'black');   

    patch('Vertices', vertices3_lla, 'Faces', faces3, ...
          'FaceColor', 'green', ...
          'FaceAlpha', 0.8, ...              
          'EdgeColor', 'black');
    hold off;
    
    hold off; % グラフの保持を解除
    
    % --- 6. グラフの見栄えを調整 ---
    title('Eagle Eye Satellite in LLA Coordinates');
    xlabel('Longitude [°E]');
    ylabel('Latitude [°N]');
    zlabel('Altitude [km]');
    
    grid on;

    % ↓↓↓ 緯度・経度と高度の単位差を補正するために、アスペクト比を調整 ↓↓↓
    km_per_deg_lat = 111.32;                % [km/°]
    km_per_deg_lon = 111.32 * cosd(lat_sat); % [km/°]
    daspect([1/km_per_deg_lon, 1/km_per_deg_lat, 1]);

    view(3);
    
    % 描画範囲を本州全体が見えるように固定
    % 本州の概略座標範囲（ECEF座標系）
    % 緯度: 約30°N-46°N, 経度: 約130°E-146°E
    % 地球半径: 約6371000m
    earth_radius = 6371000; % 地球半径 [m]
    
    % 本州の概略範囲（ECEF座標）
    % 北端: 約46°N, 南端: 約30°N
    % 東端: 約146°E, 西端: 約130°E
    lat_north = 46; lat_south = 30;
    lon_east = 146; lon_west = 130;
    
    % 北端と南端のECEF座標
    r_north = lla2ecef(lat_north, (lon_east+lon_west)/2, 0);
    r_south = lla2ecef(lat_south, (lon_east+lon_west)/2, 0);
    
    % 東端と西端のECEF座標
    r_east = lla2ecef((lat_north+lat_south)/2, lon_east, 0);
    r_west = lla2ecef((lat_north+lat_south)/2, lon_west, 0);
    
    % 表示範囲をLLA座標で設定
    margin_lon = 5; % 経度方向のマージン（5度）
    margin_lat = 5; % 緯度方向のマージン（5度）
    margin_alt = 600; % 高度方向のマージン（600km）
    
    % 本州の概略範囲をLLA座標で設定
    lon_min = lon_west - margin_lon;
    lon_max = lon_east + margin_lon;
    lat_min = lat_south - margin_lat;
    lat_max = lat_north + margin_lat;
    alt_min = 0; % 高度0km（地球表面）
    alt_max = 600; % 高度600km
    
    xlim([lon_min, lon_max]);
    ylim([lat_min, lat_max]);
    zlim([alt_min, alt_max]);

end
