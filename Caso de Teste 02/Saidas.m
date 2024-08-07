%% Resultados de Trajet�ria
% 
% figure;
% plot3(x, y, z, 'b', 'LineWidth', 2);
% xlabel('Posi��o x [m]');
% ylabel('Posi��o y [m]');
% zlabel('Posi��o z [m]');
% title('Trajet�ria Helicoidal');
% grid on;
% 
% % Posi��es ao longo do tempo
% figure;
% subplot(3, 1, 1);
% plot(t, x, 'b', 'LineWidth', 2);
% xlabel('Tempo [s]');
% ylabel('Posi��o x [m]');
% title('Posi��o ao Longo do Tempo');
% grid on;
% 
% subplot(3, 1, 2);
% plot(t, y, 'b', 'LineWidth', 2);
% xlabel('Tempo [s]');
% ylabel('Posi��o y [m]');
% grid on;
% 
% subplot(3, 1, 3);
% plot(t, z, 'b', 'LineWidth', 2);
% xlabel('Tempo [s]');
% ylabel('Posi��o z [m]');
% grid on;
% 
% % Atitude ao longo do tempo
% figure;
% subplot(3, 1, 1);
% plot(t, roll_deg, 'b', 'LineWidth', 2);
% xlabel('Tempo [s]');
% ylabel('Roll [�]');
% title('Atitude ao Longo do Tempo');
% grid on;
% 
% subplot(3, 1, 2);
% plot(t, pitch_deg, 'b', 'LineWidth', 2);
% xlabel('Tempo [s]');
% ylabel('Pitch [�]');
% grid on;
% 
% subplot(3, 1, 3);
% plot(t, yaw_deg, 'b', 'LineWidth', 2);
% xlabel('Tempo [s]');
% ylabel('Yaw [�]');
% grid on;
% 
% % Velocidade angular ao longo do tempo
% figure;
% subplot(3, 1, 1);
% plot(t, d_roll_deg, 'b', 'LineWidth', 2);
% xlabel('Tempo [s]');
% ylabel('D_Roll [�/s]');
% title('Velocidade Angular ao Longo do Tempo');
% grid on;
% 
% subplot(3, 1, 2);
% plot(t, d_pitch_deg, 'b', 'LineWidth', 2);
% xlabel('Tempo [s]');
% ylabel('D_Pitch [�/s]');
% grid on;
% 
% subplot(3, 1, 3);
% plot(t, d_yaw_deg, 'b', 'LineWidth', 2);
% xlabel('Tempo [s]');
% ylabel('D_Yaw [�/s]');
% grid on;
% 
%% Resultado dos Sensores
% 
% % Aceler�metro
% figure;
% subplot(3, 1, 1);
% plot(t, ax_n, 'b', 'LineWidth', 1.5);
% xlabel('Tempo [s]');
% ylabel('Acc_x [m/s^2]');
% title('Aceler�metro');
% grid on;
% 
% subplot(3, 1, 2);
% plot(t, ay_n, 'b', 'LineWidth', 1.5);
% xlabel('Tempo [s]');
% ylabel('Acc_y [m/s^2]');
% grid on;
% 
% subplot(3, 1, 3);
% plot(t, az_n, 'b', 'LineWidth', 1.5);
% xlabel('Tempo [s]');
% ylabel('Acc_z [m/s^2]');
% grid on;
% 
% % Girosc�pio
% figure;
% subplot(3, 1, 1);
% plot(t, p_n, 'b', 'LineWidth', 1.5);
% xlabel('Tempo [s]');
% ylabel('D_{Roll} [�/s]');
% title('Girosc�pio (rad/s)');
% grid on;
% 
% subplot(3, 1, 2);
% plot(t, q_n, 'b', 'LineWidth', 1.5);
% xlabel('Tempo [s]');
% ylabel('D_{Pitch} [�/s]');
% grid on;
% 
% subplot(3, 1, 3);
% plot(t, r_n, 'b', 'LineWidth', 1.5);
% xlabel('Tempo [s]');
% ylabel('D_{Yaw} [�/s]');
% grid on;
% 
% % Magnet�metro
% figure;
% subplot(3, 1, 1);
% plot(t, mx_n, 'b', 'LineWidth', 1.5);
% xlabel('Tempo [s]');
% ylabel('Mag x [�T]');
% title('Magnet�metro');
% grid on;
% 
% subplot(3, 1, 2);
% plot(t, my_n, 'b', 'LineWidth', 1.5);
% xlabel('Tempo [s]');
% ylabel('Mag y [�T]');
% grid on;
% 
% subplot(3, 1, 3);
% plot(t, mz_n, 'b', 'LineWidth', 1.5);
% xlabel('Tempo');
% ylabel('Mag z [�T]');
% grid on;
% 
% % Campo Magnetico WMM
% 
% figure;
% subplot(3, 1, 1);
% plot(t, hx, 'b', 'LineWidth', 1.5);
% xlabel('Tempo [s]');
% ylabel('H_x [�T]');
% title('Campo Magn�tico');
% grid on;
% 
% subplot(3, 1, 2);
% plot(t, hy, 'b', 'LineWidth', 1.5);
% xlabel('Tempo [s]');
% ylabel('H_y [�T]');
% grid on;
% 
% subplot(3, 1, 3);
% plot(t, hz, 'b', 'LineWidth', 1.5);
% xlabel('Tempo [s]');
% ylabel('H_z [�T]');
% grid on;
% 
% % Posi��es de GPS
% figure;
% subplot(3, 1, 1);
% plot(t, lat_n, 'b', 'LineWidth', 1.5);
% xlabel('Tempo [s]');
% ylabel('Latitude [�]');
% title('LLA');
% grid on;
% 
% subplot(3, 1, 2);
% plot(t, lon_n, 'b', 'LineWidth', 1.5);
% xlabel('Tempo [s]');
% ylabel('Longitude [�]');
% grid on;
% 
% subplot(3, 1, 3);
% plot(t, alt_n, 'b', 'LineWidth', 1.5);
% xlabel('Tempo [s]');
% ylabel('Altitude [m]');
% grid on;
% 
%% Resultados de Atitude
% 
% figure;
% subplot(3, 1, 1);
% hold on;
% plot(t, roll_deg, 'b', 'LineWidth', 2, 'DisplayName', 'Refer�ncia');
% plot(t, roll_triad, 'r', 'LineWidth', 1.5, 'DisplayName', 'TRIAD');
% plot(t, roll_quest, 'g', 'LineWidth', 1.5, 'DisplayName', 'QUEST');
% plot(t, roll_mh, 'm', 'LineWidth', 1.5, 'DisplayName', 'Mahony');
% plot(t, roll_ekf, 'c', 'LineWidth', 1.5, 'DisplayName', 'EKF');
% xlabel('Tempo [s]');
% ylabel('Roll [�]');
% legend('show');
% title('Atitude ao Longo do Tempo');
% grid on;
% hold off;
% 
% subplot(3, 1, 2);
% hold on;
% plot(t, pitch_deg, 'b', 'LineWidth', 2);
% plot(t, pitch_triad, 'r', 'LineWidth', 1.5);
% plot(t, pitch_quest, 'g', 'LineWidth', 1.5);
% plot(t, pitch_mh, 'm', 'LineWidth', 1.5);
% plot(t, pitch_ekf, 'c', 'LineWidth', 1.5);
% xlabel('Tempo [s]');
% ylabel('Pitch [�]');
% grid on;
% hold off;
% 
% subplot(3, 1, 3);
% hold on;
% plot(t, yaw_deg, 'b', 'LineWidth', 2);
% plot(t, yaw_triad, 'r', 'LineWidth', 1.5);
% plot(t, yaw_quest, 'g', 'LineWidth', 1.5);
% plot(t, yaw_mh, 'm', 'LineWidth', 1.5);
% plot(t, yaw_ekf, 'c', 'LineWidth', 1.5);
% xlabel('Tempo [s]');
% ylabel('Yaw [�]');
% grid on;
% hold off;
% 
%% Desempenho

% N�mero de amostras
N = length(t);

% Inicializar vari�veis para armazenar RMSE ao longo do tempo
RMSE_roll_triad = zeros(N, 1);
RMSE_pitch_triad = zeros(N, 1);
RMSE_yaw_triad = zeros(N, 1);
RMSE_roll_quest = zeros(N, 1);
RMSE_pitch_quest = zeros(N, 1);
RMSE_yaw_quest = zeros(N, 1);
RMSE_roll_mh = zeros(N, 1);
RMSE_pitch_mh = zeros(N, 1);
RMSE_yaw_mh = zeros(N, 1);
RMSE_roll_ekf = zeros(N, 1);
RMSE_pitch_ekf = zeros(N, 1);
RMSE_yaw_ekf = zeros(N, 1);

for i = 1:N
    RMSE_roll_triad(i) = sqrt(mean((roll(1:i) - roll_triad(1:i)).^2));
    RMSE_pitch_triad(i) = sqrt(mean((pitch(1:i) - pitch_triad(1:i)).^2));
    RMSE_yaw_triad(i) = sqrt(mean((yaw(1:i) - yaw_triad(1:i)).^2));

    RMSE_roll_quest(i) = sqrt(mean((roll(1:i) - roll_quest(1:i)).^2));
    RMSE_pitch_quest(i) = sqrt(mean((pitch(1:i) - pitch_quest(1:i)).^2));
    RMSE_yaw_quest(i) = sqrt(mean((yaw(1:i) - yaw_quest(1:i)).^2));

    RMSE_roll_mh(i) = sqrt(mean((roll(1:i) - roll_mh(1:i)).^2));
    RMSE_pitch_mh(i) = sqrt(mean((pitch(1:i) - pitch_mh(1:i)).^2));
    RMSE_yaw_mh(i) = sqrt(mean((yaw(1:i) - yaw_mh(1:i)).^2));
    
    RMSE_roll_ekf(i) = sqrt(mean((roll(1:i) - roll_ekf(1:i)).^2));
    RMSE_pitch_ekf(i) = sqrt(mean((pitch(1:i) - pitch_ekf(1:i)).^2));
    RMSE_yaw_ekf(i) = sqrt(mean((yaw(1:i) - yaw_ekf(1:i)).^2));
end

figure;
subplot(3, 1, 1);
hold on;
plot(t, RMSE_roll_triad, 'r','LineWidth', 1.5, 'DisplayName', 'RMSE TRIAD');
plot(t, RMSE_roll_quest, 'g','LineWidth', 1.5, 'DisplayName', 'RMSE QUEST');
plot(t, RMSE_roll_mh, 'm','LineWidth', 1.5, 'DisplayName', 'RMSE Mahony');
plot(t, RMSE_roll_ekf, 'c','LineWidth', 1.5, 'DisplayName', 'RMSE EKF');
xlabel('Tempo [s]');
ylabel('RMSE Roll');
legend('show');
title('Atitude ao Longo do Tempo');
grid on;
hold off;

subplot(3, 1, 2);
hold on;
plot(t, RMSE_pitch_triad, 'r','LineWidth', 1.5);
plot(t, RMSE_pitch_quest, 'g','LineWidth', 1.5);
plot(t, RMSE_pitch_mh, 'm','LineWidth', 1.5);
plot(t, RMSE_pitch_ekf, 'c','LineWidth', 1.5);
xlabel('Tempo [s]');
ylabel('RMSE Pitch');
grid on;
hold off;

subplot(3, 1, 3);
hold on;
plot(t, RMSE_yaw_triad, 'r','LineWidth', 1.5);
plot(t, RMSE_yaw_quest, 'g','LineWidth', 1.5);
plot(t, RMSE_yaw_mh, 'm','LineWidth', 1.5);
 plot(t, RMSE_yaw_ekf, 'c','LineWidth', 1.5);
xlabel('Tempo [s]');
ylabel('RMSE Yaw');
grid on;
hold off;
