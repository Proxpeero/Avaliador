%%%%%%%%%%%%%%%%%%%% Main Teste %%%%%%%%%%%%%%%%%%%%

% close all
% clear 
% clc
% 
% %%  Módulo 01 - Trajetória Giroscópica
% 
% h = 15; % altura de z [m]
% omega = 0.2*pi; % velocidade angular (rad/s)
% A = 1; % amplitude do movimento sinuisidal
% gamma = 0.6; % frequência do movimento sinuisidal 
% f = 1000; % numero de passos de simulação
% t_total = 80; % tempo total de simulação [s]
% 
% run Trajetoria.m % script com as informações de trajetória

%% Módulo 02 - Sensores
% 
% % Vetor de Gravidade
% g = [0; 0; 9.81]; 
% 
% % Ponto de entrada (latitude e longitude iniciais em graus)
% lat_in = -15.793889;  % Latitude inicial (Brasília)
% lon_in = -47.882778;  % Longitude inicial (Brasília)
% alt_in = 1000;        % Altitude inicial (em metros)
% 
% % Desvio padrão dos sensores
% std_acc = 0.2; % acelerometro
% std_gyro = 0.03; % giroscopio
% std_gps_LL   = 1.0e-07; % posição de GPS para latitude e longitude
% std_gps_A   = 1.0e-02; % posição de GPS para altitude
% std_vgps_LL   = 5.0e-02; % velocidade de GPS para latitude e longitude 
% std_vgps_A   = 1.0e-02; % velocidade de GPS para altitude
% std_mag = 0.8; % magnetômetro
% 
% run Sensores.m % script para geração de dados de sensores
% 
%% Módulo 03 - Algoritmos de Atitude
% 
% run TRIAD % script do algoritmo TRIAD
% 
% k_a = 1; % peso dos sensores de aceleração QUEST
% k_m = 1; % peso dos sensores de campo magnetico QUEST
% run QUEST.m % script do algoritmo QUEST
% 
% k_p = 1; % ganho proporcional do filtro Mahony
% k_i = 0; % ganho integral do filtro Mahony
% run Mahony.m % script do algoritmo Mahony
% 
% std_q = 0.001;
% run EKF.m % script do algoritmo EKF
% 
%% Módulo 04 -  Plots de Saída

run Saidas.m % script com os plots de saídas
