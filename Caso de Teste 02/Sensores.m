%%%%%%%%%%%%%%%%%%%% Geração de Sensores %%%%%%%%%%%%%%%%%%%%

%% Acelerômetro

a = [ax; ay; az];
a_b = zeros(3, length(t));
for i = 1:length(t)
    R = M_Rot(roll(i), pitch(i), yaw(i));
    a_b(:, i) = R * (a(:, i) - g);
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
    p_a = p(i); 
    q_a = q(i);
    r_a = r(i);
    gyro_b(:, i) = [p_a q_a r_a];
end

p_b = gyro_b(1, :);
q_b = gyro_b(2, :);
r_b = gyro_b(3, :);

p_n = p_b + randn(size(p_b)) * std_gyro;
q_n = q_b + randn(size(q_b)) * std_gyro;
r_n = r_b + randn(size(r_b)) * std_gyro;

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
