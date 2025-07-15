function [] = CreateStateGraphUser(ts, xs, save_count)
    global x_ 
    figure(21);
    clf;
    subplot(2, 1, 1);
    euler_angles_deg = quat2euler_deg(xs, save_count);
    plot(ts(1:end-1), euler_angles_deg(1:end-1,1)); hold on;
    plot(ts(1:end-1), euler_angles_deg(1:end-1,2));
    plot(ts(1:end-1), euler_angles_deg(1:end-1,3));
    xlabel("Time [sec]");
    ylabel("Euler Angle [-]");
    legend({"pitch","roll","yaw"}); legend;
    subplot(2, 1, 2);
    plot(ts(1:end-1), xs(1:end-1,12)); hold on;
    plot(ts(1:end-1), xs(1:end-1,13));
    plot(ts(1:end-1), xs(1:end-1,14));
    % ylim([-1, 1])
    xlabel("Time [sec]");
    ylabel("Angular Velocity [rad/s]");
    legend({"wx", "wy","wz"});
    %{
    figure(22);
    clf;
    subplot(2, 1, 1);
    plot(ts(1:end-1), tau_air(1:end-1,1)); hold on;
    plot(ts(1:end-1), tau_air(1:end-1,2));
    plot(ts(1:end-1), tau_air(1:end-1,3));
    xlabel("Time [sec]");
    ylabel("Quaternion [-]");
    legend({"pitch","roll","yaw"});
    subplot(2, 1, 2);
    plot(ts(1:end-1), xs(1:end-1,12)); hold on;
    plot(ts(1:end-1), xs(1:end-1,13));
    plot(ts(1:end-1), xs(1:end-1,14));
    % ylim([-1, 1])
    xlabel("Time [sec]");
    ylabel("Angular Velocity [rad/s]");
    legend({"wx", "wy","wz"});
    %}
end

function euler_angles_deg = quat2euler_deg(xs, save_count)
    %quat2euler_deg クォータニオンをオイラー角度（Z-Y-X順、ロール・ピッチ・ヨー）に変換し、度で出力します
    %   euler_angles_deg = quat2euler_deg(q) は、クォータニオン q を対応する
    %   オイラー角度 [ロール (phi), ピッチ (theta), ヨー (psi)] に変換し、
    %   度単位で返します。
    %   クォータニオン q は [qw, qx, qy, qz] の形式です。

    euler_angles_deg = zeros(save_count,3);

    for i = 1:save_count
        
        % 単位クォータニオンに正規化（計算の安定性のため）
        q = xs(i,8:11) ./ norm(xs(i,8:11));
        
        % クォータニオンの各要素を抽出
        qx = q(1);
        qy = q(2);
        qz = q(3);
        qw = q(4);
        
        % --- Roll (x軸まわりの回転) ---
        roll = atan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx^2 + qy^2));
        
        % --- Pitch (y軸まわりの回転) ---
        % ジンバルロック（Pitchが±90度になる特異点）をチェック
        test_pitch = 2 * (qw * qy - qz * qx);
        if abs(test_pitch) >= 1
            % ジンバルロックの場合
            pitch = sign(test_pitch) * (pi / 2);
        else
            pitch = asin(test_pitch);
        end
        
        % --- Yaw (z軸まわりの回転) ---
        yaw = atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy^2 + qz^2));
        
        % 結果をベクトルにまとめる [Yaw, Pitch, Roll]
        eul = [yaw; pitch; roll];
        
        % ラジアンから度への変換
        pitch_deg = rad2deg(pitch);
        roll_deg = rad2deg(roll);
        yaw_deg = rad2deg(yaw);
        
        % オイラー角度を度で返す
        euler_angles_deg(i,1) = pitch_deg; 
        euler_angles_deg(i,2) = roll_deg; 
        euler_angles_deg(i,3) = yaw_deg;

    end

end
