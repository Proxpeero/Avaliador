%% Trajetória Helicoidal 6DOF

% Tempo de simulação
t = linspace(0, n_turns*2*pi/omega, 1000);

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
roll  = A .* sin(gamma .* t);
pitch = A .* cos(gamma .* t);
yaw   = omega .* t;

roll_deg = wrapTo180(rad2deg(roll));
pitch_deg = mod((mod(rad2deg(pitch), 360)) + 90, 180) - 90;
yaw_deg = wrapTo180(rad2deg(yaw));

% Velocidade Angular
d_roll = A .* gamma .* cos(gamma .* t); 
d_pitch = -A .* gamma .* sin(gamma .* t); 
d_yaw = omega * ones(size(t)); 

d_roll_deg = wrapTo180(rad2deg(roll));
d_pitch_deg = mod((mod(rad2deg(pitch), 360)) + 90, 180) - 90;
d_yaw_deg = wrapTo180(rad2deg(yaw));
