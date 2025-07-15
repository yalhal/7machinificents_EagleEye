function [] = CreateBodyFigUser(utc, r, v, q, hw, mag)

    global user

    % --- 1. 基本となる直方体の寸法を定義 ---
    width1  = 0.8; % X軸方向の幅
    length1 = 0.8; % Y軸方向の長さ
    height1 = 1.4; % Z軸方向の高さ

    % --- 1.5. クォータニオンから回転行列への変換 ---
    % (a) クォータニオンqから回転行列R_qを作成
    R_q = q2dcm(q);
    
    % (b) Y軸まわりに-90度回転させる行列を作成
    theta_y1 = -90;
    R_y_minus90 = [ cosd(theta_y1),  0,  sind(theta_y1);
                    0,               1,  0;
                   -sind(theta_y1),  0,  cosd(theta_y1)];
    
    % (c) Y軸まわりに90度回転させる行列を作成
    theta_y2 = 90;
    R_y_plus90 = [ cosd(theta_y2),  0,  sind(theta_y2);
                   0,               1,  0;
                  -sind(theta_y2),  0,  cosd(theta_y2)];
    % ★3つの回転行列を結合
    % 回転は指定された順に、行列を左から掛けていく
    R_total = R_q;
    
    % --- 2. 直方体の頂点(Vertices)と面(Faces)を定義 ---
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
    
    % --- 3. 上部に載せる長方形（ソーラーパネル）の寸法と位置を定義 ---
    width2  = 3.4; % X軸方向の幅
    length2 = 1.4; % Y軸方向の長さ
    % ★直方体の上面の高さに、さらに0.05だけオフセットを追加
    x_position = (width1 / 2) + 0.01; 
    
    % --- 4. 長方形の頂点と面を定義 ---
    vertices2 = [
        x_position, -width2/2, -length2/2;
        x_position,  width2/2, -length2/2;
        x_position,  width2/2,  length2/2;
        x_position, -width2/2,  length2/2
    ];
    faces2 = [1, 2, 3, 4]; % 面は1つだけ

    % --- 4.5. -Y面に貼り付ける円の定義を追加 ---
    radius_c = 0.6 / 2; % 直径1なので半径は0.5
    num_points = 50; % 円を表現するための頂点数
    theta_c = linspace(0, 2*pi, num_points); % 0から2πまでの角度ベクトル
    % 円の頂点座標を計算 (-X平面上に作成)
    x_coords_c = radius_c * cos(theta_c);
    y_coords_c = radius_c * sin(theta_c);
    % X座標を直方体の-X面から-0.05だけずらした位置に固定
    z_coord_c  = height1/2 + 0.01; 
    z_coords_c = ones(1, num_points) * z_coord_c;
    % 頂点行列を作成
    vertices3 = [x_coords_c', y_coords_c', z_coords_c'];
    % 円の面を定義 (1からNまでの頂点を全て結ぶ)
    faces3 = 1:num_points;



    % which is current target
    if user.current_target_index > length(user.use_targets)
        % all target used
        target_eci_nomed = [0; 0; 0];
        return
    elseif user.current_target_index < length(user.use_targets)
        target_latlon = user.use_targets{user.current_target_index};
        target_eci = ecef2eci(lla2ecef(target_latlon(1), target_latlon(2), 0), utc);
        target_eci_nomed = (target_eci - r) ./ norm(target_eci - r) .* 2.5;
    end
    
    % --- 4.9. 回転 ---
    [rows, cols] = size(vertices1');
    vertices1_rotated = zeros(rows, cols);
    for i = 1:cols
        vertices1_rotated(:,i) = R_total * vertices1(i,:)';
    end
    [rows, cols] = size(vertices2');
    vertices2_rotated = zeros(rows, cols);
    for i = 1:cols
        vertices2_rotated(:,i) = R_total * vertices2(i,:)';
    end
    [rows, cols] = size(vertices3');
    vertices3_rotated = zeros(rows, cols);
    for i = 1:cols
        vertices3_rotated(:,i) = R_total * vertices3(i,:)';
    end

    r_nomed = r ./ norm(r) .* 3;
    v_nomed = v ./ norm(v) .* 3;
    A = [1, 0, 0, cos(pi/4)*cos(pi/4);
         0, 1, 0, sin(pi/4)*cos(pi/4);
         0, 0, 1, sin(pi/4)];
    hw_vec = A * hw;
    hw_nomed  = hw_vec ./ norm(hw_vec) .* 3;
    mag_nomed = mag ./ norm(mag) .* 3;

    x_b = R_total * [1;0;0] .* 1.5; 
    y_b = R_total * [0;1;0] .* 1.5; 
    z_b = R_total * [0;0;1] .* 1.5; 

    % --- 5. プロットの実行 ---
    figure(22); % 新しい図ウィンドウを作成
    clf;
    
    % patch関数で直方体を描画
    patch('Vertices', vertices1_rotated', 'Faces', faces1, ...
          'FaceColor', [1.0, 0.65, 0.0], ... % オレンジ色のRGB値
          'FaceAlpha', 0.8, ...              
          'EdgeColor', 'black');             
    
    hold on; % 現在のグラフを保持して、上書きせずに次のプロットを追加
    
    % patch関数で上部の長方形（ソーラーパネル）を描画
    patch('Vertices', vertices2_rotated', 'Faces', faces2, ...
          'FaceColor', [0.10, 0.10, 0.44], ... % ★色を青色のRGB値に変更
          'FaceAlpha', 0.75, ...              
          'EdgeColor', 'black');   

    patch('Vertices', vertices3_rotated', 'Faces', faces3, ...
          'FaceColor', 'green', ...
          'FaceAlpha', 0.8, ...              
          'EdgeColor', 'black');

    
    
    quiver3(0,0,0,x_b(1),x_b(2),x_b(3));
    text(x_b(1), x_b(2), x_b(3), '  x_{body}');
    quiver3(0,0,0,y_b(1),y_b(2),y_b(3));
    text(y_b(1), y_b(2), y_b(3), '  y_{body}');
    quiver3(0,0,0,z_b(1),z_b(2),z_b(3));
    text(z_b(1), z_b(2), z_b(3), '  z_{body}');
    quiver3(-r_nomed(1),-r_nomed(2),-r_nomed(3),r_nomed(1),r_nomed(2),r_nomed(3),'AutoScaleFactor',1);
    text(-r_nomed(1), -r_nomed(2), -r_nomed(3), '  r_{nomed}');
    quiver3(0,0,0,v_nomed(1),v_nomed(2),v_nomed(3));
    text(v_nomed(1), v_nomed(2), v_nomed(3), '  v_{nomed}');
    %quiver3(0,0,0,hw_nomed(1),hw_nomed(2),hw_nomed(3));
    %text(hw_nomed(1), hw_nomed(2), hw_nomed(3), '  hw_{nomed}');
    quiver3(0,0,0,mag_nomed(1),mag_nomed(2),mag_nomed(3));
    text(mag_nomed(1), mag_nomed(2), mag_nomed(3), '  B_{nomed}');
    % quiver3(0,0,0,target_eci_nomed(1),target_eci_nomed(2),target_eci_nomed(3));

    hold off;
    
    
    % --- 6. グラフの見栄えを調整 ---
    title('Eagle Eye');
    xlabel('X_{ECI}');
    ylabel('Y_{ECI}');
    zlabel('Z_{ECI}');
    
    grid on;
    axis equal; % 各軸のスケールを等しくして、形状の歪みをなくす
    view(3);
    daspect([1 1 1]);
    %set(gca,'CameraUpVector',[1 0 0],'CameraUpVectorMode','manual');
    
    % 描画範囲を調整して全体が見えるようにする
    xlim([-2.5, 2.5]);
    ylim([-2.5, 2.5]);
    zlim([-2.5, 2.5]);
    
    % disp('直方体と長方形のプロットが完了しました。');
end
