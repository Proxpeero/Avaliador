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
q_init = E2Q(roll_init, pitch_init, yaw_init);
E = Q2E(q_init);
roll_init = E(1); 
pitch_init = E(2); 
yaw_init = E(3);
state_vector = [q_init(1), q_init(2), q_init(3), q_init(4), gyro_bias(1), gyro_bias(2), gyro_bias(3)];

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

bx = 0.5500;
bz = 0.8351;
    
X1 = [yaw_init*deg2rad;0];
P1 = zeros(2);
Q1 = diag([1e-6, 1e-8]);
R1 = 0.1;
Z1 = zeros(L);
estimate_yaw = zeros(L, 1);
mag_yaw = zeros(L, 1);

%% EKF
for k=L:1
    T = abs(GAP_TIME(k,1));
    if(k > 1)
        Time(k, 1) = Time(k-1, 1) + T;
    end

    
    % Cálculo das orientações a partir dos sensores
    [acc_roll(k, 1), acc_pitch(k, 1), mag_yaw(k, 1)] = A2E(acc(k, :), mag(k,:));
    acc_roll(k) = acc_roll(k, 1) * rad2deg;
    acc_pitch(k) = acc_pitch(k, 1) * rad2deg;
    mag_yaw(k) = mag_yaw(k, 1) * rad2deg;
    
    % Verificação do vetor Z
    if k > size(Z, 2)
        error('O índice k excede o número de colunas em Z.');
    end
    Z(k, :) = [acc(k, :) / norm(acc(k, :)), mag(k, :) / norm(mag(k, :))];
    
    % Atualização do vetor de estado
    gyro_x_bias_current = state_vector(5);
    gyro_y_bias_current = state_vector(6);
    gyro_z_bias_current = state_vector(7);
    w_truth = [gyro(k, 1)-gyro_x_bias_current, gyro(k, 2)-gyro_y_bias_current, gyro(k, 3)-gyro_z_bias_current];
    
    % Construção da matriz omega
    omega = [0, -w_truth(1), -w_truth(2), -w_truth(3);
                w_truth(1), 0, w_truth(3), -w_truth(2);
                w_truth(2), -w_truth(3), 0, w_truth(1);
                w_truth(3), w_truth(2), -w_truth(1), 0];
      
    quat = [state_vector(1); state_vector(2); state_vector(3); state_vector(4)];
    q_new = quat + 0.5 * T * omega * quat;
    w_bias_new = [gyro_x_bias_current; gyro_y_bias_current; gyro_z_bias_current];
    next_state_vector = [q_new; w_bias_new];

    % Normalização do quaternio
    next_state_vector(1:4) = next_state_vector(1:4) / norm(next_state_vector(1:4));
    
    % Construção da matriz de transição de estado
    line = [quat(2), quat(3), quat(4);
            -quat(1), quat(4), -quat(3);
            -quat(4), -quat(1), quat(2);
            quat(3), -quat(2), -quat(1)];
    Ak = [omega, line;
          zeros(3, 7)];
    F = eye(7) + 0.5 * T * Ak;
    
    P_next = F * P * F' + Q;
    
    % Construção da matriz de observação H
    Hb1 = next_state_vector(1)*bx-next_state_vector(3)*bz;
    Hb2 = next_state_vector(2)*bx+next_state_vector(4)*bz;
    Hb3 = -next_state_vector(3)*bx-next_state_vector(1)*bz;
    Hb4 = -next_state_vector(4)*bx+next_state_vector(2)*bz;
    H = 2 * [next_state_vector(3), -next_state_vector(4),  next_state_vector(1), -next_state_vector(2), 0, 0, 0;
            -next_state_vector(2), -next_state_vector(1), -next_state_vector(4), -next_state_vector(3), 0, 0, 0;
            -next_state_vector(1),  next_state_vector(2),  next_state_vector(3), -next_state_vector(4), 0, 0, 0;
            Hb1,    Hb2,    Hb3,    Hb4,    0, 0, 0;
            Hb4,    -Hb3,   Hb2,    -Hb1,   0, 0, 0;
            -Hb3,   -Hb4,   Hb1,    Hb2,    0, 0, 0;];
    
    SP = H * P_next * H' + R;
    SP_inv = inv(SP); % Use inv instead of ^
    
    K = P_next * H' * SP_inv;
    
    % Atualização do vetor de estado
    X_ = next_state_vector(1:4);
    hk = [-2*(X_(2)*X_(4)-X_(1)*X_(3));
          -2*(X_(1)*X_(2)+X_(3)*X_(4));
          -(X_(1)^2+X_(4)^2-X_(2)^2-X_(3)^2);
          (X_(1)^2+X_(2)^2-X_(3)^2-X_(4)^2)*bx + 2*(X_(2)*X_(4) - X_(1)*X_(3))*bz;
          2*(X_(2)*X_(3)-X_(1)*X_(4))*bx + 2*(X_(1)*X_(2)-X_(3)*X_(4))*bz;
          2*(X_(1)*X_(3)+X_(2)*X_(4))*bx + (X_(1)^2-X_(2)^2-X_(3)^2+X_(4)^2)*bz];
    
    S = Z(k, :)' - hk;
    state_vector = next_state_vector + K * S;
    state_vector(1:4) = state_vector(1:4) / norm(state_vector(1:4));
    
    P = (eye(7) - K * H) * P_next;
    
    % Estimativas de orientação
    estimate_q = [state_vector(1), state_vector(2), state_vector(3), state_vector(4)];
    E_2 = Q2E(estimate_q);
    estimate_roll = E_2(1);
    estimate_pitch = E_2(2);
    estimate_yaw = E_2(3);
end

figure;
plot(t, acc_roll, t, estimate_roll);
xlabel('t / s')
ylabel('roll')
title('roll');

figure;
plot(t, acc_pitch, t, estimate_pitch);
xlabel('t / s')
ylabel('pitch')
title('pitch');

figure;
plot(t, mag_yaw, t, estimate_yaw);
xlabel('t / s')
ylabel('yaw')
title('yaw');