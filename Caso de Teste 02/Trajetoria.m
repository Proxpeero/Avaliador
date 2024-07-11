%% Trajetória Giroscópica 3DOF

% Tempo de simulação
t = linspace(0, t_total, f);

% Posição
x = zeros(size(t));
y = zeros(size(t));
z = ones(size(t)) * h;

% Velocidade
vx = zeros(size(t));
vy = zeros(size(t));
vz = zeros(size(t));

% Aceleração
ax = zeros(size(t));
ay = zeros(size(t));
az = zeros(size(t));

% Ângulos de Atitude 
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

d_roll_deg = wrapTo180(rad2deg(d_roll));
d_pitch_deg = mod((mod(rad2deg(d_pitch), 360)) + 90, 180) - 90;
d_yaw_deg = wrapTo180(rad2deg(d_yaw));
