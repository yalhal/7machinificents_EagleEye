function [K, S] = ControlLQR(A, B, Q, R)
    n = size(A, 1);
    
    K = [1, 5]; 
    S = Q; % Sの初期値
    
    max_iter = 50;
    tolerance = 1e-8; 
    
    
    for i = 1:max_iter
        S_prev = S;
    
        % a. リアプノフ方程式を解く
        % (A-BK)'S + S(A-BK) + K'RK + Q = 0  を S について解く
        A_cl = A - B * K; % 閉ループ行列
        Q_lyap = -(K' * R * K + Q); % リアプノフ方程式の右辺項
        
        % リアプノフ方程式を行列演算で解ける形に変形
        % (I kron A_cl' + A_cl' kron I) * s_vec = -q_vec
        M = kron(eye(n), A_cl') + kron(A_cl', eye(n));
        s_vec = M \ Q_lyap(:); % vec(Q_lyap) = Q_lyap(:)
        S = reshape(s_vec, n, n);
    
        % b. ゲインを更新
        K = R \ (B' * S); % inv(R) * B' * S と同じ
    
        % c. 収束判定
        if norm(S - S_prev, 'fro') < tolerance
            % fprintf('収束しました (反復回数: %d)\n', i);
            break;
        end
    end
    
    if i == max_iter
        warning('最大反復回数に達しました。解が収束していない可能性があります。');
    end

end