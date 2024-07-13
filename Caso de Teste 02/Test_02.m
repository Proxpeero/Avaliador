%%%%%%%%%%%%%%%%%%%% Main Teste %%%%%%%%%%%%%%%%%%%%

% close all
% clear 
% clc
% 
% %%  M�dulo 01 - Trajet�ria Girosc�pica
% 
% h = 15; % altura de z [m]
% omega = 0.2*pi; % velocidade angular (rad/s)
% A = 1; % amplitude do movimento sinuisidal
% gamma = 0.6; % frequ�ncia do movimento sinuisidal 
% f = 1000; % numero de passos de simula��o
% t_total = 80; % tempo total de simula��o [s]
% 
% run Trajetoria.m % script com as informa��es de trajet�ria

%% M�dulo 02 - Sensores
% 
% % Vetor de Gravidade
% g = [0; 0; 9.81]; 
% 
% % Ponto de entrada (latitude e longitude iniciais em graus)
% lat_in = -15.793889;  % Latitude inicial (Bras�lia)
% lon_in = -47.882778;  % Longitude inicial (Bras�lia)
% alt_in = 1000;        % Altitude inicial (em metros)
% 
% % Desvio padr�o dos sensores
% std_acc = 0.2; % acelerometro
% std_gyro = 0.03; % giroscopio
% std_gps_LL   = 1.0e-07; % posi��o de GPS para latitude e longitude
% std_gps_A   = 1.0e-02; % posi��o de GPS para altitude
% std_vgps_LL   = 5.0e-02; % velocidade de GPS para latitude e longitude 
% std_vgps_A   = 1.0e-02; % velocidade de GPS para altitude
% std_mag = 0.8; % magnet�metro
% 
% run Sensores.m % script para gera��o de dados de sensores
% 
%% M�dulo 03 - Algoritmos de Atitude
% 
% run TRIAD % script do algoritmo TRIAD
% 
% k_a = 1; % peso dos sensores de acelera��o QUEST
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
%% M�dulo 04 -  Plots de Sa�da

run Saidas.m % script com os plots de sa�das
