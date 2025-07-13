function [] = CreateBodyFigUser(q, utc)
    % --- 1. 基本となる直方体の寸法を定義 ---
    width1  = 0.8; % x_body軸方向の幅
    length1 = 0.8; % y_body軸方向の長さ
    height1 = 1.4; % z_body軸方向の高さ

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
    length2 = 3.4; % y_body軸方向の長さ
    height2  = 1.4; % z_body軸方向の高さ
    % ★直方体の上面の高さに、さらに0.05だけオフセットを追加
    x_position = (width1 / 2) + 0.05; 
    
    % --- 4. 長方形の頂点と面をボディ座標系で定義 ---
    vertices2 = [
        x_position, -length2/2, -height2/2;
        x_position, -length2/2,  height2/2;
        x_position,  length2/2,  height2/2;
        x_position,  length2/2, -height2/2
    ];
    faces2 = [1, 2, 3, 4]; % 面は1つだけ

    % --- 4.5. -Y面に貼り付ける円の定義をボディ座標系で追加 ---
    radius_c = 0.6 / 2; % 直径1なので半径は0.5
    num_points = 50; % 円を表現するための頂点数
    theta_c = linspace(0, 2*pi, num_points); % 0から2πまでの角度ベクトル
    % 円の頂点座標を計算 (+Z平面上に作成)
    x_coords_c = radius_c * cos(theta_c);
    y_coords_c = radius_c * sin(theta_c);
    % Z座標を直方体の+Z面から+0.05だけずらした位置に固定
    z_coord_c = width1/2 + 0.05; 
    z_coords_c = ones(1, num_points) * z_coord_c;
    % 頂点行列を作成
    vertices3 = [x_coords_c', y_coords_c', z_coords_c'];
    % 円の面を定義 (1からNまでの頂点を全て結ぶ)
    faces3 = 1:num_points;
    
    % --- 4.9. 回転 ---
    vertices1_rotated = vertices1*R_total';
    vertices2_rotated = vertices2*R_total';
    vertices3_rotated = vertices3*R_total';

    % --- 5. プロットの実行 ---
    figure(22); % 新しい図ウィンドウを作成
    clf;
    
    % patch関数で直方体を描画
    patch('Vertices', vertices1_rotated, 'Faces', faces1, ...
          'FaceColor', [1.0, 0.65, 0.0], ... % オレンジ色のRGB値
          'FaceAlpha', 0.8, ...              
          'EdgeColor', 'black');             
    
    hold on; % 現在のグラフを保持して、上書きせずに次のプロットを追加
    
    % patch関数で上部の長方形（ソーラーパネル）を描画
    patch('Vertices', vertices2_rotated, 'Faces', faces2, ...
          'FaceColor', [0.10, 0.10, 0.44], ... % ★色を青色のRGB値に変更
          'FaceAlpha', 0.75, ...              
          'EdgeColor', 'black');   

    patch('Vertices', vertices3_rotated, 'Faces', faces3, ...
          'FaceColor', 'green', ...
          'FaceAlpha', 0.8, ...              
          'EdgeColor', 'black');
    hold off;
    
    hold off; % グラフの保持を解除
    
    % --- 6. グラフの見栄えを調整 ---
    title('Eagle Eye');
    xlabel('←軌道方向');
    % ylabel('Y軸');
    zlabel('鉛直上向→');
    
    grid on;
    axis equal; % 各軸のスケールを等しくして、形状の歪みをなくす
    view(3);
    daspect([1 1 1]);
    
    % 描画範囲を調整して全体が見えるようにする
    xlim([-2.5, 2.5]);
    ylim([-2.5, 2.5]);
    zlim([-2.5, 2.5]);
    
    % disp('直方体と長方形のプロットが完了しました。');
end
