%% Resultados de Trajetória
% 
% % Atitude ao longo do tempo
% figure;
% subplot(3, 1, 1);
% plot(t, roll_deg, 'b', 'LineWidth', 2);
% xlabel('Tempo [s]');
% ylabel('Roll [°]');
% title('Atitude ao Longo do Tempo');
% grid on;
% 
% subplot(3, 1, 2);
% plot(t, pitch_deg, 'b', 'LineWidth', 2);
% xlabel('Tempo [s]');
% ylabel('Pitch [°]');
% grid on;
% 
% subplot(3, 1, 3);
% plot(t, yaw_deg, 'b', 'LineWidth', 2);
% xlabel('Tempo [s]');
% ylabel('Yaw [°]');
% grid on;
% 
% % Velocidade angular ao longo do tempo
% figure;
% subplot(3, 1, 1);
% plot(t, d_roll_deg, 'b', 'LineWidth', 2);
% xlabel('Tempo [s]');
% ylabel('D_Roll [°/s]');
% title('Velocidade Angular ao Longo do Tempo');
% grid on;
% 
% subplot(3, 1, 2);
% plot(t, d_pitch_deg, 'b', 'LineWidth', 2);
% xlabel('Tempo [s]');
% ylabel('D_Pitch [°/s]');
% grid on;
% 
% subplot(3, 1, 3);
% plot(t, d_yaw_deg, 'b', 'LineWidth', 2);
% xlabel('Tempo [s]');
% ylabel('D_Yaw [°/s]');
% grid on;
% 
%% Resultado dos Sensores
% 
% % Acelerômetro
% figure;
% subplot(3, 1, 1);
% plot(t, ax_n, 'b');
% xlabel('Tempo [s]');
% ylabel('Acc_x [m/s^2]');
% title('Acelerômetro');
% grid on;
% 
% subplot(3, 1, 2);
% plot(t, ay_n, 'b');
% xlabel('Tempo [s]');
% ylabel('Acc_y [m/s^2]');
% grid on;
% 
% subplot(3, 1, 3);
% plot(t, az_n, 'b');
% xlabel('Tempo [s]');
% ylabel('Acc_z [m/s^2]');
% grid on;
% 
% % Giroscópio
% figure;
% subplot(3, 1, 1);
% plot(t, d_roll_n, 'b');
% xlabel('Tempo [s]');
% ylabel('D_{Roll} [°/s]');
% title('Giroscópio (rad/s)');
% grid on;
% 
% subplot(3, 1, 2);
% plot(t, d_pitch_n, 'b');
% xlabel('Tempo [s]');
% ylabel('D_{Pitch} [°/s]');
% grid on;
% 
% subplot(3, 1, 3);
% plot(t, d_yaw_n, 'b');
% xlabel('Tempo [s]');
% ylabel('D_{Yaw} [°/s]');
% grid on;
% 
% % Magnetômetro
% figure;
% subplot(3, 1, 1);
% plot(t, mx_n, 'b');
% xlabel('Tempo [s]');
% ylabel('Mag x [µT]');
% title('Magnetômetro');
% grid on;
% 
% subplot(3, 1, 2);
% plot(t, my_n, 'b');
% xlabel('Tempo [s]');
% ylabel('Mag y [µT]');
% grid on;
% 
% subplot(3, 1, 3);
% plot(t, mz_n, 'b');
% xlabel('Tempo');
% ylabel('Mag z [µT]');
% grid on;
% 
% % Campo Magnetico WMM
% 
% figure;
% subplot(3, 1, 1);
% plot(t, hx_n, 'b');
% xlabel('Tempo [s]');
% ylabel('H_x [µT]');
% title('Campo Magnético');
% grid on;
% 
% subplot(3, 1, 2);
% plot(t, hy_n, 'b');
% xlabel('Tempo [s]');
% ylabel('H_y [µT]');
% grid on;
% 
% subplot(3, 1, 3);
% plot(t, hz_n, 'b');
% xlabel('Tempo [s]');
% ylabel('H_z [µT]');
% grid on;
% 
% % Posições de GPS
% figure;
% subplot(3, 1, 1);
% plot(t, lat_n, 'b');
% xlabel('Tempo [s]');
% ylabel('Latitude [°]');
% title('LLA');
% grid on;
% 
% subplot(3, 1, 2);
% plot(t, lon_n, 'b');
% xlabel('Tempo [s]');
% ylabel('Longitude [°]');
% grid on;
% 
% subplot(3, 1, 3);
% plot(t, alt_n, 'b');
% xlabel('Tempo [s]');
% ylabel('Altitude [m]');
% grid on;
% 
%% Resultados de Atitude

figure;
subplot(3, 1, 1);
hold on;
plot(t, roll_deg, 'b', 'DisplayName', 'Referência');
plot(t, roll_triad, 'r', 'DisplayName', 'TRIAD');
plot(t, roll_quest, 'g', 'DisplayName', 'QUEST');
plot(t, roll_mh, 'k', 'DisplayName', 'Mahony');
% plot(t, roll_ekf, 'k', 'DisplayName', 'EKF');
xlabel('Tempo [s]');
ylabel('Roll [°]');
legend('show');
title('Atitude ao Longo do Tempo');
grid on;
hold off;

subplot(3, 1, 2);
hold on;
plot(t, pitch_deg, 'b');
plot(t, pitch_triad, 'r');
plot(t, pitch_quest, 'g');
plot(t, pitch_mh, 'k');
% plot(t, pitch_ekf, 'k', 'DisplayName', 'EKF');
xlabel('Tempo [s]');
ylabel('Pitch [°]');
grid on;
hold off;

subplot(3, 1, 3);
hold on;
plot(t, yaw_deg, 'b');
plot(t, yaw_triad, 'r');
plot(t, yaw_quest, 'g');
plot(t, yaw_mh, 'k');
% plot(t, yaw_ekf, 'k', 'DisplayName', 'EKF');
xlabel('Tempo [s]');
ylabel('Yaw [°]');
grid on;
hold off;

%% Desempenho
% 
% % Número de amostras
% N = length(t);
% 
% % Inicializar variáveis para armazenar RMSE ao longo do tempo
% RMSE_roll_triad = zeros(N, 1);
% RMSE_pitch_triad = zeros(N, 1);
% RMSE_yaw_triad = zeros(N, 1);
% RMSE_roll_quest = zeros(N, 1);
% RMSE_pitch_quest = zeros(N, 1);
% RMSE_yaw_quest = zeros(N, 1);
% RMSE_roll_mh = zeros(N, 1);
% RMSE_pitch_mh = zeros(N, 1);
% RMSE_yaw_mh = zeros(N, 1);
% 
% for i = 1:N
%     RMSE_roll_triad(i) = sqrt(mean((roll(1:i) - roll_triad(1:i)).^2));
%     RMSE_pitch_triad(i) = sqrt(mean((pitch(1:i) - pitch_triad(1:i)).^2));
%     RMSE_yaw_triad(i) = sqrt(mean((yaw(1:i) - yaw_triad(1:i)).^2));
% 
%     RMSE_roll_quest(i) = sqrt(mean((roll(1:i) - roll_quest(1:i)).^2));
%     RMSE_pitch_quest(i) = sqrt(mean((pitch(1:i) - pitch_quest(1:i)).^2));
%     RMSE_yaw_quest(i) = sqrt(mean((yaw(1:i) - yaw_quest(1:i)).^2));
% 
%     RMSE_roll_mh(i) = sqrt(mean((roll(1:i) - roll_mh(1:i)).^2));
%     RMSE_pitch_mh(i) = sqrt(mean((pitch(1:i) - pitch_mh(1:i)).^2));
%     RMSE_yaw_mh(i) = sqrt(mean((yaw(1:i) - yaw_mh(1:i)).^2));
% end
% 
% figure;
% subplot(3, 1, 1);
% hold on;
% plot(t, RMSE_roll_triad, 'r', 'DisplayName', 'RMSE TRIAD');
% plot(t, RMSE_roll_quest, 'g', 'DisplayName', 'RMSE QUEST');
% plot(t, RMSE_roll_mh, 'k', 'DisplayName', 'RMSE Mahony');
% % plot(t, roll_ekf, 'k', 'DisplayName', 'EKF');
% xlabel('Tempo [s]');
% ylabel('Roll [°]');
% legend('show');
% title('Atitude ao Longo do Tempo');
% grid on;
% hold off;
% 
% subplot(3, 1, 2);
% hold on;
% plot(t, RMSE_pitch_triad, 'r', 'DisplayName', 'RMSE TRIAD');
% plot(t, RMSE_pitch_quest, 'g', 'DisplayName', 'RMSE QUEST');
% plot(t, RMSE_pitch_mh, 'k', 'DisplayName', 'RMSE Mahony');
% % plot(t, pitch_ekf, 'k', 'DisplayName', 'EKF');
% xlabel('Tempo [s]');
% ylabel('Pitch [°]');
% grid on;
% hold off;
% 
% subplot(3, 1, 3);
% hold on;
% plot(t, RMSE_yaw_triad, 'r', 'DisplayName', 'RMSE TRIAD');
% plot(t, RMSE_yaw_quest, 'g', 'DisplayName', 'RMSE QUEST');
% plot(t, RMSE_yaw_mh, 'k', 'DisplayName', 'RMSE Mahony');
% % plot(t, yaw_ekf, 'k', 'DisplayName', 'EKF');
% xlabel('Tempo [s]');
% ylabel('Yaw [°]');
% grid on;
% hold off;
