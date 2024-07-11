%%%%%%%%%%%% EKF %%%%%%%%%%%

rad2deg = 180/pi;
deg2rad = pi/180;

acc = [ax_n, ay_n, az_n];
gyro_bias = [0.01; 0.01; 0.01];
gyro = [d_roll_n - gyro_bias(1), d_pitch_n - gyro_bias(2), d_yaw_n - gyro_bias(3)];
mag = [mx_n, my_n, mz_n];

w_process_noise = 1e-6 * ones(1, 4);
w_bias_process_noise = 1e-8 * ones(1,3);
Q = diag([w_process_noise, w_bias_process_noise]);

a_measure_noise = 1e-1 * ones(1, 3);
m_measure_noise = 0.2 * ones(1, 3);
R = diag([a_measure_noise, m_measure_noise]);

[roll_init, pitch_init, yaw_init] = A2E(acc(1, :), mag(1, :));
roll_init1 = roll_init * rad2deg;
pitch_init1 = pitch_init * rad2deg;
yaw_init1 = yaw_init * rad2deg;

q_init = zeros(4, 1);
q_init1 = E2Q(roll_init1, pitch_init1, yaw_init1);
E = Q2E(q_init1);
roll_init2 = E(1); 
pitch_init2 = E(2); 
yaw_init2 = E(3);
state_vector = [q_init1(1), q_init1(2), q_init1(3), q_init1(4), gyro_bias(1), gyro_bias(2), gyro_bias(3)];

p1 = 1 * ones(1, 4);
p2 = 1 * ones(1, 3);
P = diag([p1, p2]);

L = 1000;

Z = zeros(L, 6);
estimate_roll = zeros(L, 1);
acc_roll = zeros(L, 1);
estimate_pitch = zeros(L, 1);
acc_pitch = zeros(L, 1);

%%

hx = 0.5500;
hz = 0.8351;
    
X1 = [yaw_init2*deg2rad; 0];
P1 = zeros(2);
Q1 = diag([1e-6, 1e-8]);
R1 = 0.1;
Z1 = zeros(L);
estimate_yaw = zeros(L, 1);
mag_yaw = zeros(L, 1);
Att = zeros(3, length(t));

%% EKF
for k=L:1
    T = abs(GAP_TIME(k,1));
    if(k > 1)
        Time(k, 1) = Time(k-1, 1) + T;
    end

    
    % C�lculo das orienta��es a partir dos sensores
    [acc_roll(k, 1), acc_pitch(k, 1), mag_yaw(k, 1)] = A2E(acc(k, :), mag(k,:));
    acc_roll(k) = acc_roll(k, 1) * rad2deg;
    acc_pitch(k) = acc_pitch(k, 1) * rad2deg;
    mag_yaw(k) = mag_yaw(k, 1) * rad2deg;
    
    Z(k, :) = [acc(k, :) / norm(acc(k, :)), mag(k, :) / norm(mag(k, :))];
    
    % Atualiza��o do vetor de estado
    gyro_x_bias_current = state_vector(5);
    gyro_y_bias_current = state_vector(6);
    gyro_z_bias_current = state_vector(7);
    w_truth = [gyro(k, 1)- gyro_x_bias_current, gyro(k, 2)- gyro_y_bias_current, gyro(k, 3)- gyro_z_bias_current];
    
    % Constru��o da matriz omega
    omega = [0, -w_truth(1), -w_truth(2), -w_truth(3);
                w_truth(1), 0, w_truth(3), -w_truth(2);
                w_truth(2), -w_truth(3), 0, w_truth(1);
                w_truth(3), w_truth(2), -w_truth(1), 0];
      
    quat = [state_vector(1); state_vector(2); state_vector(3); state_vector(4)];
    q_new = quat + 0.5 * T * omega * quat;
    w_bias_new = [gyro_x_bias_current; gyro_y_bias_current; gyro_z_bias_current];
    next_state_vector = [q_new; w_bias_new];

    % Normaliza��o do quaternio
    next_state_vector(1:4) = next_state_vector(1:4) / norm(next_state_vector(1:4));
    
    % Constru��o da matriz de transi��o de estado
    line = [quat(2), quat(3), quat(4);
            -quat(1), quat(4), -quat(3);
            -quat(4), -quat(1), quat(2);
            quat(3), -quat(2), -quat(1)];
    Ak = [omega, line;
          zeros(3, 7)];
    F = eye(7) + 0.5 * T * Ak;
    
    P_next = F * P * F' + Q;
    
    % Constru��o da matriz de observa��o H
    Hb1 = next_state_vector(1)*hx-next_state_vector(3)*hz;
    Hb2 = next_state_vector(2)*hx+next_state_vector(4)*hz;
    Hb3 = -next_state_vector(3)*hx-next_state_vector(1)*hz;
    Hb4 = -next_state_vector(4)*hx+next_state_vector(2)*hz;
    H = 2 * [next_state_vector(3), -next_state_vector(4),  next_state_vector(1), -next_state_vector(2), 0, 0, 0;
            -next_state_vector(2), -next_state_vector(1), -next_state_vector(4), -next_state_vector(3), 0, 0, 0;
            -next_state_vector(1),  next_state_vector(2),  next_state_vector(3), -next_state_vector(4), 0, 0, 0;
            Hb1,    Hb2,    Hb3,    Hb4,    0, 0, 0;
            Hb4,    -Hb3,   Hb2,    -Hb1,   0, 0, 0;
            -Hb3,   -Hb4,   Hb1,    Hb2,    0, 0, 0;];
    
    SP = H * P_next * H' + R;
    SP_inv = inv(SP); % Use inv instead of ^
    
    K = P_next * H' * SP_inv;
    
    % Atualiza��o do vetor de estado
    X_ = next_state_vector(1:4);
    hk = [-2*(X_(2)*X_(4)-X_(1)*X_(3));
          -2*(X_(1)*X_(2)+X_(3)*X_(4));
          -(X_(1)^2+X_(4)^2-X_(2)^2-X_(3)^2);
          (X_(1)^2+X_(2)^2-X_(3)^2-X_(4)^2)*hx + 2*(X_(2)*X_(4) - X_(1)*X_(3))*hz;
          2*(X_(2)*X_(3)-X_(1)*X_(4))*hx + 2*(X_(1)*X_(2)-X_(3)*X_(4))*hz;
          2*(X_(1)*X_(3)+X_(2)*X_(4))*hx + (X_(1)^2-X_(2)^2-X_(3)^2+X_(4)^2)*hz];
    
    S = Z(k, :)' - hk;
    state_vector = next_state_vector + K * S;
    state_vector(:, k) = state_vector(1:4) / norm(state_vector(1:4));
    
    P = (eye(7) - K * H) * P_next;
    
    % Estimativas de orienta��o
    q = [state_vector(1,:), state_vector(1,:), state_vector(1,:), state_vector(1,:)];
    Att(:, k) = Q2E(q);
end

% roll_ekf = wrapTo180(rad2deg(Att(:, 1)));
% pitch_ekf = mod((mod(rad2deg(Att(:, 2)), 360)) + 90, 180) - 90;
% yaw_ekf = wrapTo180(rad2deg(Att(:, 3)));
% 
% %% Plots das Sa�das
% 
% % Atitude
% figure;
% subplot(3, 1, 1);
% hold on;
% plot(t, roll_deg, 'b', 'DisplayName', 'Refer�ncia');
% plot(t, roll_ekf, 'r', 'DisplayName', 'EKF');
% xlabel('Tempo [s]');
% ylabel('Roll [�]');
% legend('show');
% title('�ngulos de Atitude');
% grid on;
% hold off;
% 
% subplot(3, 1, 2);
% hold on;
% plot(t, pitch_deg, 'b');
% plot(t, pitch_ekf, 'r');
% xlabel('Tempo [s]');
% ylabel('Pitch [�]');
% grid on;
% hold off;
% 
% subplot(3, 1, 3);
% hold on;
% plot(t, yaw_deg, 'b');
% plot(t, yaw_ekf, 'r');
% xlabel('Tempo [s]');
% ylabel('Yaw [�]');
% grid on;
% hold off;