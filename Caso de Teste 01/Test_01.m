%%%%%%%%%%%%%%%%%%%% Main Teste %%%%%%%%%%%%%%%%%%%%

close all
clear 
clc

%%  M�dulo 01 - Trajet�ria Helicoidal

r = 1; % raio da h�lice
h = 20; % altura da h�lice
omega = 0.2*pi; % velocidade angular (rad/s)
n_turns = 5; % n�mero de voltas da h�lice
A = 1; % amplitude do movimento sinuisidal
gamma = 0.6; % frequ�ncia do movimento sinuisidal 
f = 1000; % numero de passos de simula��o

run Trajetoria.m % script com as informa��es de trajet�ria

%% M�dulo 02 - Sensores

% Vetor de Gravidade
g = [0; 0; 9.81]; 

% Ponto de entrada (latitude e longitude iniciais em graus)
lat_in = -15.793889;  % Latitude inicial (Bras�lia)
lon_in = -47.882778;  % Longitude inicial (Bras�lia)
alt_in = 1000;        % Altitude inicial (em metros)

% Desvio padr�o dos sensores
std_acc = 0.2; % acelerometro
std_gyro = 0.5; % giroscopio
std_gps_LL   = 1.0e-07; % posi��o de GPS para latitude e longitude
std_gps_A   = 1.0e-02; % posi��o de GPS para altitude
std_vgps_LL   = 5.0e-02; % velocidade de GPS para latitude e longitude 
std_vgps_A   = 1.0e-02; % velocidade de GPS para altitude
std_mag = 1; % magnet�metro

run Sensores.m % script para gera��o de dados de sensores

%% M�dulo 02 - Algoritmos de Atitude

% run TRIAD % script do algoritmo TRIAD
% 
% k_a = 0.5; % peso dos sensores de acelera��o QUEST
% k_m = 1; % peso dos sensores de campo magnetico QUEST
% run QUEST.m % script do algoritmo QUEST
% 
% k_p = 10; % ganho proporcional do filtro Mahony
% k_i = 10; % ganho integral do filtro Mahony
% run Mahony.m % script do algoritmo Mahony

dt = t(2) - t(1);
sigma_ap = 0; 
sigma_mp = 0;

sigma_a = 5.0e-02;
sigma_g = 1.0e-03;
sigma_m = 0.8;

run EKF_02.m % script do algoritmo EKF

%% M�dulo 04 -  Plots de Sa�da
% 
% run Saidas.m % script com os plots de sa�das
