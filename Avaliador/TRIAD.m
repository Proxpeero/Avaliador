%% TRIAD 

Att = zeros(3, length(t));

for i = 1:length(t)
    
    acc = [ax_n(i) ay_n(i) az_n(i)];
    mag = [mx_n(i) my_n(i) mz_n(i)];
    h = [hx(i) hy(i) hz(i)];
    
    w_1 = acc ./ norm(acc);
    w_2 = mag ./ norm(mag);
    
    q_b = w_1;
    r_b = cross(w_1, w_2) ./ norm(cross(w_1, w_2));
    s_b = cross(q_b, r_b);
    M_b = [q_b; r_b; s_b];
    
    v_1 = g' ./ norm(g');
    v_2 = h ./ norm(h);
    
    q_r = v_1;
    r_r = cross(v_1, v_2) ./ norm(cross(v_1, v_2));
    s_r = cross(q_r, r_r);
    M_r = [q_r; r_r; s_r];
    
    R = M_b * transpose(M_r);
    
    Att(:, i) = M2E(R);
end

roll_triad = wrapTo180(rad2deg(Att(1, :)));
pitch_triad = mod((mod(rad2deg(Att(2, :)), 360)) + 90, 180) - 90;
yaw_triad = wrapTo180(rad2deg(Att(3, :)));

%% Plots das Saídas
% 
% figure;
% subplot(3, 1, 1);
% hold on;
% plot(t, roll_deg, 'b', 'DisplayName', 'Referência');
% plot(t, roll_triad, 'r', 'DisplayName', 'TRIAD');
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
% plot(t, pitch_triad, 'r');
% xlabel('Tempo [s]');
% ylabel('Pitch [°]');
% grid on;
% hold off;
% 
% subplot(3, 1, 3);
% hold on;
% plot(t, yaw_deg, 'b');
% plot(t, yaw_triad, 'r');
% xlabel('Tempo [s]');
% ylabel('Yaw [°]');
% grid on;
% hold off;
