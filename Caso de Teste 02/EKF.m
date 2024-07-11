%% EKF

x_k = [1; 0; 0; 0; 0; 0; 0; 0; 0; 0];
P_k = eye(10);

x_hist = zeros(10, length(t));
Att = zeros(3, length(t));

for i = 1:length(t)
    
    acc = [ax_n(i) ay_n(i) az_n(i)];
    gyro = [d_roll_n(i) d_pitch_n(i) d_yaw_n(i)];
    mag = [mx_n(i) my_n(i) mz_n(i)];
    vel_GPS = [vx_n(i) vy_n(i) vz_n(i)];
    pos_GPS = [lat_n(i) lon_n(i) alt_n(i)];
    heading = [hx_n(i) hy_n(i) hz_n(i)];
    
    % Predição
    [x_k1_, P_k1_] = predicao(acc, gyro, dt, x_k, P_k, g', mrp_q, mrp_p, mrp_v);
    
    % Atualização
    [x_k1, P_k1] = atualizacao(acc, mag, heading, vel_GPS, pos_GPS, g, x_k1_, P_k1_, mcm_a, mcm_m, mcm_v, mcm_p);
    
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
title('Atitude ao Longo do Tempo');
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
