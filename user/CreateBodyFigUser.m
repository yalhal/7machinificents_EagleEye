function [] = CreateBodyFigUser(q, utc, r_ecef)
    % --- 1. 基本となる直方体の寸法を定義（スケール調整） ---
    scale = 100000; % 衛星のスケール（実際のサイズを100000倍に拡大）
    width1  = 0.8 * scale; % x_body軸方向の幅
    length1 = 0.8 * scale; % y_body軸方向の長さ
    height1 = 1.4 * scale; % z_body軸方向の高さ

    % --- 1.5. クォータニオンから回転行列への変換 ---
    % ボディ座標系から慣性座標系(ECI)への変換行列
    R_body_to_eci = q2dcm(q);

    % 慣性座標系（ECI）から地球固定座標系（ECEF）への変換
    timehack = [year(utc), month(utc), day(utc), hour(utc), minute(utc), second(utc)];
    gast = siderealtime(timehack);
    R_eci_to_ecef = CoordinateRotation("z", gast);

    % 合成回転: ボディ座標系 → 地球固定座標系（ECEF）
    R_body_to_ecef = R_eci_to_ecef * R_body_to_eci;
    
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

    % --- 4.5. +Z面に貼り付ける円の定義をボディ座標系で追加 ---
    radius_c = 0.6 / 2 * scale; % 直径1なので半径は0.5
    num_points = 50; % 円を表現するための頂点数
    theta_c = linspace(0, 2*pi, num_points); % 0から2πまでの角度ベクトル
    % 円の頂点座標を計算 (+Z平面上に作成)
    x_coords_c = radius_c * cos(theta_c);
    y_coords_c = radius_c * sin(theta_c);
    % Z座標を直方体の+Z面から+0.05だけずらした位置に固定
    % height1 は Z軸方向の高さなので、+height1/2 が上面の中心になります。
    z_coord_c = height1/2 + 0.05 * scale;
    z_coords_c = ones(1, num_points) * z_coord_c;
    % 頂点行列を作成
    vertices3 = [x_coords_c', y_coords_c', z_coords_c'];
    % 円の面を定義 (1からNまでの頂点を全て結ぶ)
    faces3 = 1:num_points;
    
    % --- 4.9. 回転と位置の適用 ---
    % ボディ座標系からECEF座標系への変換
    % verticesは行ベクトルとして定義されているため、回転行列の転置を右から掛ける
    vertices1_rotated = vertices1 * R_body_to_ecef';
    vertices2_rotated = vertices2 * R_body_to_ecef';
    vertices3_rotated = vertices3 * R_body_to_ecef';
    
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
    
    % patch関数で直方体を描画（LLA座標系）
    patch('Vertices', vertices1_lla, 'Faces', faces1, ...
          'FaceColor', [1.0, 0.65, 0.0], ... % オレンジ色のRGB値
          'FaceAlpha', 0.8, ...              
          'EdgeColor', 'black');             
    
    hold on; % 現在のグラフを保持して、上書きせずに次のプロットを追加
    
    % patch関数で上部の長方形（ソーラーパネル）を描画（LLA座標系）
    patch('Vertices', vertices2_lla, 'Faces', faces2, ...
          'FaceColor', [0.10, 0.10, 0.44], ... % ★色を青色のRGB値に変更
          'FaceAlpha', 0.75, ...              
          'EdgeColor', 'black');   

    % patch関数で円を描画（LLA座標系）
    patch('Vertices', vertices3_lla, 'Faces', faces3, ...
          'FaceColor', 'green', ...
          'FaceAlpha', 0.8, ...              
          'EdgeColor', 'black');
    
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
    % ... (描画範囲の設定は変更なし) ...
    lon_min = 125;
    lon_max = 150;
    lat_min = 30;
    lat_max = 50;
    alt_min = 0;
    alt_max = 600;
    
    xlim([lon_min, lon_max]);
    ylim([lat_min, lat_max]);
    zlim([alt_min, alt_max]);
    
    % targets.jsonのターゲットを矩形領域としてプロット
    hold on;
    global targets_
    global TARGET_CENTER_INDEX TARGET_NE_INDEX TARGET_NW_INDEX TARGET_SW_INDEX TARGET_SE_INDEX
    
    % 現在撮影中のターゲットインデックスを取得
    [current_obs_index, ~, ~, ~, ~, ~, ~, ~] = IsValidObservation();
    
    for i = 1:length(targets_)
        target = targets_{i};
        center_latlon = target{TARGET_CENTER_INDEX};
        ne_latlon = target{TARGET_NE_INDEX};
        nw_latlon = target{TARGET_NW_INDEX};
        sw_latlon = target{TARGET_SW_INDEX};
        se_latlon = target{TARGET_SE_INDEX};
        
        % 矩形の4隅の座標をLLA座標で設定
        rect_vertices = [
            ne_latlon(2), ne_latlon(1), 0; % 北東
            nw_latlon(2), nw_latlon(1), 0; % 北西
            sw_latlon(2), sw_latlon(1), 0; % 南西
            se_latlon(2), se_latlon(1), 0  % 南東
        ];
        
        % 撮影中のターゲットかどうかで色を決定
        if i == current_obs_index
            % 撮影中: 赤色
            rect_color = 'red';
            edge_color = 'red';
        else
            % 撮影していない: 緑色
            rect_color = 'green';
            edge_color = 'green';
        end
        
        % 矩形をプロット
        patch('Vertices', rect_vertices, 'Faces', [1 2 3 4], ...
              'FaceColor', rect_color, 'FaceAlpha', 0.3, 'EdgeColor', edge_color);
    end
    
    % 衛星の位置をマーカーで表示
    plot3(lon_sat, lat_sat, alt_sat/1000, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'red');
    
    % ボディ座標系の+Z方向（カメラ方向）の半直線を描画
    % ボディ座標系での光軸方向ベクトル (列ベクトル)
    los_body = [0; 0; 1]; % +Z方向
    
    % ボディ座標系からECEF座標系への変換
    los_ecef = R_body_to_ecef * los_body;
    
    % 光軸の長さを設定（地上に届くまで）
    los_length = alt_sat * 2; % 十分な長さを設定
    
    % 光軸の終点を計算（ECEF座標系）
    los_end_ecef = r_ecef + los_ecef * los_length;
    
    % 光軸の終点をLLA座標に変換
    [los_end_lat, los_end_lon, los_end_alt] = ecef2lla(los_end_ecef);
    
    % 光軸を描画（赤い線）
    plot3([lon_sat, los_end_lon], [lat_sat, los_end_lat], [alt_sat/1000, los_end_alt/1000], 'r-', 'LineWidth', 2);
    
    hold off;

end