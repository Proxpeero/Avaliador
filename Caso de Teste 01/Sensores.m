%%%%%%%%%%%%%%%%%%%% Geração de Sensores %%%%%%%%%%%%%%%%%%%%

%% Acelerômetro

a = [ax; ay; az];
a_b = zeros(3, length(t));
for i = 1:length(t)
    R = M_Rot(roll(i), pitch(i), yaw(i));
    a_b(:, i) = R * (a(:, i) + g);
end

ax_b = a_b(1, :);
ay_b = a_b(2, :);
az_b = a_b(3, :);

ax_n = ax_b + randn(size(ax_b)) * std_acc;
ay_n = ay_b + randn(size(ay_b)) * std_acc;
az_n = az_b + randn(size(az_b)) * std_acc;

%% Giroscópio

gyro_b = zeros(3, length(t));

for i = 2:length(t)
    R = M_Rot(roll(i), pitch(i), yaw(i));
    omega = [d_roll(i); d_pitch(i); d_yaw(i)];
    gyro_b(:, i) = R * omega;
end

d_roll_b  = gyro_b(1, :);
d_pitch_b = gyro_b(2, :);
d_yaw_b   = gyro_b(3, :);

d_roll_n  = d_roll_b  + randn(size(d_roll_b))  * std_gyro;
d_pitch_n = d_pitch_b + randn(size(d_pitch_b)) * std_gyro;
d_yaw_n   = d_yaw_b   + randn(size(d_yaw_b))   * std_gyro;

%% GPS
% Conversão para latitude e longitude (aproximação)
m2deg_lat = 1 / 111000; % 1 metro em graus de latitude
m2deg_lon = 1 / (111000 * cosd(lat_in)); % 1 metro em graus de longitude

% Calculando latitude e longitude
lat = lat_in + y * m2deg_lat;
lon = lon_in + x * m2deg_lon;
alt = alt_in + z;

lat_n = lat + randn(size(lat)) * std_gps_LL; 
lon_n = lon + randn(size(lon)) * std_gps_LL; 
alt_n = alt + randn(size(alt)) * std_gps_A;

vx_n = vx + randn(size(vx)) * std_vgps_LL; 
vy_n = vy + randn(size(vy)) * std_vgps_LL; 
vz_n = vz + randn(size(vz)) * std_vgps_A;

%% Magnetômetro

m_b = zeros(3, length(t));
h = zeros(3, length(t));
for i = 1:length(t)
    [XYZ, H, D, I, F] = wrldmagm(alt(i), lat(i), lon(i), decyear(2015,7,4),'2015');
    R = M_Rot(roll(i), pitch(i), yaw(i));
    h(:, i) = XYZ;
    m_b(:, i) = R * XYZ;
end

hx = h(1, :); 
hy = h(2, :); 
hz = h(3, :); 

mx = m_b(1, :); 
my = m_b(2, :); 
mz = m_b(3, :); 

mx_n = mx + randn(size(mx)) * std_mag;
my_n = my + randn(size(my)) * std_mag;
mz_n = mz + randn(size(mz)) * std_mag;
