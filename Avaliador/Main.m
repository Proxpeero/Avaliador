%%%%%%%%%%%%%%%%%%%% Main Teste %%%%%%%%%%%%%%%%%%%%

clear
clc

%% Entrada 01 (Condições iniciais)

% % Parâmetros da trajetória helicoidal
% r = 5; % raio da hélice
% h = 20; % altura da hélice
% n_turns = 5; % número de voltas da hélice
% omega = 2*pi; % velocidade angular (rad/s)
% 
% % Parâmetro de tempo
% t = linspace(0, n_turns*2*pi/omega, 5000);

%% Entrada 02 (Condições iniciais)

% Parâmetros da trajetória helicoidal
r = 0; % raio da hélice
h = 10; % altura da hélice
n_turns = 3; % número de voltas da hélice
omega = 0.5*pi; % velocidade angular (rad/s)

% Parâmetro de tempo
t = linspace(0, n_turns*2*pi/omega, 1000);

%% Entrada 03 (Condições iniciais)

% % Parâmetros da trajetória helicoidal
% r = 3; % raio da hélice
% h = 25; % altura da hélice
% n_turns = 5; % número de voltas da hélice
% omega = 3*pi; % velocidade angular (rad/s)
% 
% % Parâmetro de tempo
% t = linspace(0, n_turns*2*pi/omega, 10000);

%% Script de Trajetória e Sensores

run script_trajetoria.m % script com as informações de trajetória
% run script_trajetoria_02.m % script com as informações de trajetória
run script_sensores.m % script para geração de dados de sensores

%% Scripts de Atitude e Posição

% run script_TRIAD % script do algoritmo TRIAD
% run script_QUEST.m % script do algoritmo QUEST
% run script_Mahony.m % script do algoritmo Mahony
% run script_Madgwick.m % script do algoritmo Madgwick (ruim)
run script_EKF.m % script do algoritmo EKF

%% Script de Plots de Saída

% run script_saidas.m % script com os plots de saídas
