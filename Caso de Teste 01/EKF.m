%% EKF

x_k = [1; 0; 0; 0; 0; 0; 0];
P_k = eye(7);

x_hist = zeros(7, length(t));
Att = zeros(3, length(t));

for i = 1:length(t)
    
    acc = [ax_n(i) ay_n(i) az_n(i)];
    gyro = [d_roll_n(i) d_pitch_n(i) d_yaw_n(i)];
    mag = [mx_n(i) my_n(i) mz_n(i)];
    heading = [hx(i) hy(i) hz(i)];
    
    % Predição
    [x_k1_, P_k1_] = EKF_Prediction(gyro, dt, x_k, P_k, mrp_q);
    
    % Atualização
    [x_k1, P_k1] = EKF_Update(acc, mag, heading, g, x_k1_, P_k1_, mcm_a, mcm_m);
    
    % Atualização dos estados
    x_k = x_k1_;
    P_k = P_k1;
    
    % Armazenamento dos resultados
    x_hist(:, i) = x_k;
    
    q = [x_hist(1,:) x_hist(2,:) x_hist(3,:) x_hist(4,:)];
    
    Att(:, i) = Q2E(q);
end

roll_ekf = wrapTo180(rad2deg(Att(1, :)));
pitch_ekf = mod((mod(rad2deg(Att(2, :)), 360)) + 90, 180) - 90;
yaw_ekf = wrapTo180(rad2deg(Att(3, :)));

%% Plots das Saídas

% Atitude
figure;
subplot(3, 1, 1);
hold on;
plot(t, roll_deg, 'b', 'DisplayName', 'Referência');
plot(t, roll_ekf, 'r', 'DisplayName', 'EKF');
xlabel('Tempo [s]');
ylabel('Roll [°]');
legend('show');
title('Ângulos de Atitude');
grid on;
hold off;

subplot(3, 1, 2);
hold on;
plot(t, pitch_deg, 'b');
plot(t, pitch_ekf, 'r');
xlabel('Tempo [s]');
ylabel('Pitch [°]');
grid on;
hold off;

subplot(3, 1, 3);
hold on;
plot(t, yaw_deg, 'b');
plot(t, yaw_ekf, 'r');
xlabel('Tempo [s]');
ylabel('Yaw [°]');
grid on;
hold off;
