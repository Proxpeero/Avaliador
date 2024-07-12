%% QUEST 

Att = zeros(3, length(t));
for i = 1:length(t)
    
    acc = [ax_n(i) ay_n(i) az_n(i)];
    mag = [mx_n(i) my_n(i) mz_n(i)];
    h   = [hx(i) hy(i) hz(i)];
    
    B = k_a * (acc' * g') + k_m * (mag' * h);
    K_11 = B + B' - eye(3) * trace(B);
    K_22 = trace(B);
    K_21 = k_a * cross(acc, g') + k_m * cross(mag, h);
    K_12 = K_21';
    
    K = [K_11, K_12; K_21, K_22];
    
    [V, D] = eig(K);
    lambdas = diag(D);
    
    index = lambdas == max(lambdas);
    q_opt = V(:, index);
    q = [q_opt(4), q_opt(1:3, 1)'];
    q = q / sqrt(q * q');
    q = [q(1), -q(2), -q(3), -q(4)];
    
    Att(:, i) = Q2E(q);
end

roll_quest = wrapTo180(rad2deg(Att(1, :)));
pitch_quest = mod((mod(rad2deg(Att(2, :)), 360)) + 90, 180) - 90;
yaw_quest = wrapTo180(rad2deg(Att(3, :)));

%% Plots das Saídas
% figure;
% subplot(3, 1, 1);
% hold on;
% plot(t, roll_deg, 'b', 'DisplayName', 'Referência');
% plot(t, roll_quest, 'r', 'DisplayName', 'QUEST');
% xlabel('Tempo [s]');
% ylabel('Roll [°]');
% legend('show');
% title('Atitude ao Longo do Tempo');
% grid on;
% hold off;
% 
% subplot(3, 1, 2);
% hold on;
% plot(t, pitch_deg, 'b', 'DisplayName', 'Referência');
% plot(t, pitch_quest, 'r', 'DisplayName', 'QUEST');
% xlabel('Tempo [s]');
% ylabel('Pitch [°]');
% legend('show');
% grid on;
% hold off;
% 
% subplot(3, 1, 3);
% hold on;
% plot(t, yaw_deg, 'b', 'DisplayName', 'Referência');
% plot(t, yaw_quest, 'r', 'DisplayName', 'QUEST');
% xlabel('Tempo [s]');
% ylabel('Yaw [°]');
% legend('show');
% grid on;
% hold off;
