%%%%%%%%%%%%%%%%%%%% Main Teste %%%%%%%%%%%%%%%%%%%%

clear
clc

%% Entrada 01 (Condi��es iniciais)

% % Par�metros da trajet�ria helicoidal
% r = 5; % raio da h�lice
% h = 20; % altura da h�lice
% n_turns = 5; % n�mero de voltas da h�lice
% omega = 2*pi; % velocidade angular (rad/s)
% 
% % Par�metro de tempo
% t = linspace(0, n_turns*2*pi/omega, 5000);

%% Entrada 02 (Condi��es iniciais)

% Par�metros da trajet�ria helicoidal
r = 0; % raio da h�lice
h = 10; % altura da h�lice
n_turns = 3; % n�mero de voltas da h�lice
omega = 0.5*pi; % velocidade angular (rad/s)

% Par�metro de tempo
t = linspace(0, n_turns*2*pi/omega, 1000);

%% Entrada 03 (Condi��es iniciais)

% % Par�metros da trajet�ria helicoidal
% r = 3; % raio da h�lice
% h = 25; % altura da h�lice
% n_turns = 5; % n�mero de voltas da h�lice
% omega = 3*pi; % velocidade angular (rad/s)
% 
% % Par�metro de tempo
% t = linspace(0, n_turns*2*pi/omega, 10000);

%% Script de Trajet�ria e Sensores

run script_trajetoria.m % script com as informa��es de trajet�ria
% run script_trajetoria_02.m % script com as informa��es de trajet�ria
run script_sensores.m % script para gera��o de dados de sensores

%% Scripts de Atitude e Posi��o

% run script_TRIAD % script do algoritmo TRIAD
% run script_QUEST.m % script do algoritmo QUEST
% run script_Mahony.m % script do algoritmo Mahony
% run script_Madgwick.m % script do algoritmo Madgwick (ruim)
run script_EKF.m % script do algoritmo EKF

%% Script de Plots de Sa�da

% run script_saidas.m % script com os plots de sa�das
