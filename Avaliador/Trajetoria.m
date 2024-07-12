%% Modelos de Trajetórias

clear
clc

model = input('Por favor, insira um número entre 1 e 8: ');

% Entradas gerais: t, r, omega, lambda, gamma

r = 5;
h = 10;
n_turns = 3;
lambda = 5;
gamma = 0.1;
omega = 0.5*pi;
k = 0.1;

t = linspace(0, n_turns*2*pi/omega, 1000);

switch model
    case 1
        disp('Trajetória helicoidal 01');
        
        % Posição
        x = r * cos(omega * t);
        y = r * sin(omega * t);
        z = (h/(2*pi)) * t;

        % Velocidade
        vx = -r * omega * sin(omega * t);
        vy = r * omega * cos(omega * t);
        vz = (h/(2*pi)) * ones(size(t));

        % Aceleração
        ax = -r * omega^2 * cos(omega * t);
        ay = -r * omega^2 * sin(omega * t);
        az = zeros(size(t));

        % Atitude 
        roll = zeros(size(t));
        pitch = zeros(size(t));
        yaw = omega * t;

        roll_deg = wrapTo180(rad2deg(roll));
        pitch_deg = mod((mod(rad2deg(pitch), 360)) + 90, 180) - 90;
        yaw_deg = wrapTo180(rad2deg(yaw));

        % Velocidade angular
        d_roll = zeros(size(t)); 
        d_pitch = zeros(size(t)); 
        d_yaw = omega * ones(size(t));
        
	case 2
        disp('Trajetória helicoidal 02');
        
        % Posição
        x = r * cos(omega * t);
        y = r * sin(omega * t);
        z = (h/(2*pi)) * t;

        % Velocidade
        vx = -r * omega * sin(omega * t);
        vy = r * omega * cos(omega * t);
        vz = (h/(2*pi)) * ones(size(t));

        % Aceleração
        ax = -r * omega^2 * cos(omega * t);
        ay = -r * omega^2 * sin(omega * t);
        az = zeros(size(t));

        % Atitude 
        roll  = sin(2 .* pi .* gamma .* t);
        pitch = cos(2 .* pi .* gamma .* t);
        yaw   = omega .* t;
        
        roll_deg = wrapTo180(rad2deg(roll));
        pitch_deg = mod((mod(rad2deg(pitch), 360)) + 90, 180) - 90;
        yaw_deg = wrapTo180(rad2deg(yaw));
        
        % Velocidade angular
        d_roll = zeros(size(t)); 
        d_pitch = zeros(size(t)); 
        d_yaw = omega * ones(size(t));
        
	case 3
        disp('Trajetória Circular');
        
        % Posição
        x = r * cos(omega * t);
        y = r * sin(omega * t);
        z = zeros(size(t));
        
        % Atitude 
        roll = zeros(size(t));
        pitch = zeros(size(t));
        yaw = omega * t;
        
        roll_deg = wrapTo180(rad2deg(roll));
        pitch_deg = mod((mod(rad2deg(pitch), 360)) + 90, 180) - 90;
        yaw_deg = wrapTo180(rad2deg(yaw));
        
	case 4
        disp('Trajetória Sinusoidal');
        
        % Entradas movimento sinusoidal
        A_xyz = [1, 2, 1];
        omega_xyz = [1, 0.5, 1];
        phi_xyz = [pi, pi/2, pi];

        A_euler = [pi/6, pi/12, pi/10];
        omega_euler = [0.4, 0.3, 0.2];
        phi_euler = [pi/3, pi/4, pi/6];

        % Posição
        x = A_xyz(1) * sin(omega_xyz(1) * t + phi_xyz(1));
        y = A_xyz(2) * sin(omega_xyz(2) * t + phi_xyz(2));
        z = A_xyz(3) * sin(omega_xyz(3) * t + phi_xyz(3));

        % Atitude
        roll = A_euler(1) * sin(omega_euler(1) * t + phi_euler(1));
        pitch = A_euler(2) * sin(omega_euler(2) * t + phi_euler(2));
        yaw = A_euler(3) * sin(omega_euler(3) * t + phi_euler(3));

        roll_deg = wrapTo180(rad2deg(roll));
        pitch_deg = mod((mod(rad2deg(pitch), 360)) + 90, 180) - 90;
        yaw_deg = wrapTo180(rad2deg(yaw));
        
	case 5
        disp('Trajetória em Lemniscata');

        A = sqrt(2);
        A_euler = [pi/6, pi/12, pi/10];
        omega_euler = [0.4, 0.3, 0.2];
        phi_euler = [pi/3, pi/4, pi/6];

        % Posição
        x = A * cos(t);
        y = A * sin(t) .* cos(t);
        z = zeros(size(t));

        % Atitude
        roll = A_euler(1) * sin(omega_euler(1) * t + phi_euler(1));
        pitch = A_euler(2) * sin(omega_euler(2) * t + phi_euler(2));
        yaw = A_euler(3) * sin(omega_euler(3) * t + phi_euler(3));
        
        roll_deg = wrapTo180(rad2deg(roll));
        pitch_deg = mod((mod(rad2deg(pitch), 360)) + 90, 180) - 90;
        yaw_deg = wrapTo180(rad2deg(yaw));
        
	case 6
        disp('Trajetória Serpentina');
        
        % Posição
        x = t;
        y = lambda * sin(omega * t) .* cos(omega * t / 2);
        z = lambda * sin(omega * t);

        % Atitude
        roll = zeros(size(t));
        pitch = [0, atan2(diff(z), diff(x))];
        yaw = zeros(size(t));
        yaw(end) = yaw(end-1);

        roll_deg = wrapTo180(rad2deg(roll));
        pitch_deg = mod((mod(rad2deg(pitch), 360)) + 90, 180) - 90;
        yaw_deg = wrapTo180(rad2deg(yaw));
        
	case 7
        disp('Trajetória em Espiral');
        
        % Posição
        x = (r + k .* t) .* cos(omega * t);
        y = (r + k .* t) .* sin(omega * t);
        z = (h/(2*pi)) .* t;

        % Atitude 
        roll = zeros(size(t));
        pitch = zeros(size(t));
        yaw = omega * t;

        roll_deg = wrapTo180(rad2deg(roll));
        pitch_deg = mod((mod(rad2deg(pitch), 360)) + 90, 180) - 90;
        yaw_deg = wrapTo180(rad2deg(yaw));
        
	case 8
        disp('Trajetória Giroscópica');
        
        % Posição
        x = r * cos(omega * t);
        y = r * sin(omega * t);
        z = h * cos(2*omega * t);

        % Atitude 
        roll = zeros(size(t));
        pitch = zeros(size(t));
        yaw = omega * t;

        roll_deg = wrapTo180(rad2deg(roll));
        pitch_deg = mod((mod(rad2deg(pitch), 360)) + 90, 180) - 90;
        yaw_deg = wrapTo180(rad2deg(yaw));
        
    otherwise
        disp('Entrada inválida');
end

%% Saídas (Plots)

% Trajetória em 3D
figure;
plot3(x, y, z);
xlabel('X');
ylabel('Y');
zlabel('Z');
title('Trajetória');
grid on;

% Atitude (roll, pitch, yaw)
figure;
subplot(3, 1, 1);
plot(t, roll_deg);
xlabel('Tempo');
ylabel('roll');
title('Atitude');
grid on;

subplot(3, 1, 2);
plot(t, pitch_deg);
xlabel('Tempo');
ylabel('pitch');
grid on;

subplot(3, 1, 3);
plot(t, yaw_deg);
xlabel('Tempo');
ylabel('yaw');
grid on;

% Posição (xyz)
figure;
subplot(3, 1, 1);
plot(t, x);
xlabel('Tempo');
ylabel('Posição x');
title('Posição');
grid on;

subplot(3, 1, 2);
plot(t, y);
xlabel('Tempo');
ylabel('Posição y');
grid on;

subplot(3, 1, 3);
plot(t, z);
xlabel('Tempo');
ylabel('Posição z');
grid on;