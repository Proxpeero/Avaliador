%% EKF
dt = 0.03;
x_k = [1; 0; 0; 0; 0; 0; 0]; % Vetor de estado inicial
P_k = eye(7); % Matriz de covari�ncia inicial

x_hist = zeros(7, length(t)); % Inicializa��o da matriz de hist�rico de estados
Att = zeros(3, length(t)); % Inicializa��o da matriz de �ngulos de atitude

for i = 1:length(t)
    % Leitura dos sensores
    acc = [ax_n(i); ay_n(i); az_n(i)];
    gyro = [d_roll_n(i); d_pitch_n(i); d_yaw_n(i)];
    mag = [mx_n(i); my_n(i); mz_n(i)];
    h   = [hx(i) hy(i) hz(i)];

    % Predi��o
    [x_k1_, P_k1_] = EKF_Prediction(gyro, dt, x_k, P_k, std_q, std_gyro);
    
    % Atualiza��o
    [x_k1, P_k1] = EKF_Update(acc, mag, x_k1_, P_k1_, std_acc, std_mag, g, h);
    
    % Atualiza��o dos estados
    x_k = x_k1;
    P_k = P_k1;
    
    % Armazenamento dos resultados
    x_hist(:, i) = x_k;
    
    % Extra��o do quat�rnion atual para convers�o
    q = x_hist(1:4, i); % Quat�rnion atual
    
    % Convers�o do quat�rnion para �ngulos de Euler
    Att(:, i) = Q2E(q);
end

roll_ekf = wrapTo180(rad2deg(Att(1, :)));
pitch_ekf = mod((mod(rad2deg(Att(2, :)), 360)) + 90, 180) - 90;
yaw_ekf = wrapTo180(rad2deg(Att(3, :)));

%% Plots das Sa�das

% Atitude
figure;
subplot(3, 1, 1);
hold on;
plot(t, roll_deg, 'b', 'DisplayName', 'Refer�ncia');
plot(t, roll_ekf, 'r', 'DisplayName', 'EKF');
xlabel('Tempo [s]');
ylabel('Roll [�]');
legend('show');
title('�ngulos de Atitude');
grid on;
hold off;

subplot(3, 1, 2);
hold on;
plot(t, pitch_deg, 'b');
plot(t, pitch_ekf, 'r');
xlabel('Tempo [s]');
ylabel('Pitch [�]');
grid on;
hold off;

subplot(3, 1, 3);
hold on;
plot(t, yaw_deg, 'b');
plot(t, yaw_ekf, 'r');
xlabel('Tempo [s]');
ylabel('Yaw [�]');
grid on;
hold off;
