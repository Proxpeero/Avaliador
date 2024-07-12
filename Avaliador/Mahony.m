%% Mahony 

Att = zeros(3, length(t));
eInt = [0 0 0];
q = [1 0 0 0];
dt = t(2) - t(1);

for i = 1:length(t)
    
    acc = [ax_n(i), ay_n(i), az_n(i)];
    mag = [mx_n(i), my_n(i), mz_n(i)];
    gyro = [d_roll_n(i), d_pitch_n(i), d_yaw_n(i)];
    
    acc = acc / norm(acc);
    mag = mag / norm(mag);
    
    v = [2*(q(2)*q(4) - q(1)*q(3));
         2*(q(1)*q(2) + q(3)*q(4));
         q(1)^2 - q(2)^2 - q(3)^2 + q(4)^2];
     
    h = [2*mag(1)*(0.5 - q(3)^2 - q(4)^2) + 2*mag(2)*(q(2)*q(3) - q(1)*q(4)) + 2*mag(3)*(q(2)*q(4) + q(1)*q(3));
         2*mag(1)*(q(2)*q(3) + q(1)*q(4)) + 2*mag(2)*(0.5 - q(2)^2 - q(4)^2) + 2*mag(3)*(q(3)*q(4) - q(1)*q(2));
         2*mag(1)*(q(2)*q(4) - q(1)*q(3)) + 2*mag(2)*(q(3)*q(4) + q(1)*q(2)) + 2*mag(3)*(0.5 - q(2)^2 - q(3)^2)]; 
        
    b = [sqrt((h(1)^2) + (h(2)^2));
         0;
         h(3)];

    w = [2*b(1)*(0.5 - q(3)^2 - q(4)^2) + 2*b(3)*(q(2)*q(4) - q(1)*q(3));
         2*b(1)*(q(2)*q(3) - q(1)*q(4)) + 2*b(3)*(q(1)*q(2) + q(3)*q(4));
         2*b(1)*(q(1)*q(3) + q(2)*q(4)) + 2*b(3)*(0.5 - q(2)^2 - q(3)^2)];
     
    e = cross(acc, v) + cross(mag, w);
    eInt = eInt + e * dt;
    gyro = gyro + k_p * e + k_i * eInt;
    qDot = 0.5 * QQ(-q, [0 gyro(1) gyro(2) gyro(3)]);

    q = q + qDot' * dt;
    q = q / norm(q);
    Att(:, i) = Q2E(q);
end

roll_mh = wrapTo180(rad2deg(Att(1, :)));
pitch_mh = mod((mod(rad2deg(Att(2, :)), 360)) + 90, 180) - 90;
yaw_mh = wrapTo180(rad2deg(Att(3, :)));

%% Plots das Saídas
% 
% figure;
% subplot(3, 1, 1);
% hold on;
% plot(t, roll_deg, 'b', 'DisplayName', 'Referência');
% plot(t, roll_mh, 'r', 'DisplayName', 'Mahony');
% xlabel('Tempo [s]');
% ylabel('Roll [°]');
% legend('show');
% title('Atitude ao Longo do Tempo');
% grid on;
% hold off;
% 
% subplot(3, 1, 2);
% hold on;
% plot(t, pitch_deg, 'b');
% plot(t, pitch_mh, 'r');
% xlabel('Tempo [s]');
% ylabel('Pitch [°]');
% grid on;
% hold off;
% 
% subplot(3, 1, 3);
% hold on;
% plot(t, yaw_deg, 'b');
% plot(t, yaw_mh, 'r');
% xlabel('Tempo [s]');
% ylabel('Yaw [°]');
% grid on;
% hold off;
