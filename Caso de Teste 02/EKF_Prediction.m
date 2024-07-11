function [x_k1_,P_k1_] = predicao(acc, gyro, dt, x_k, P_k, g, mrp_q, mrp_p, mrp_v)
%% Saídas

   % quaternion = q0, q1, q2, q3;
   % velocidade = vel_N, vel_E, vel_D;
   % posição = pos_N, pos_E, pos_D;
   
%% delta do ângulo calculado pelo giroscopio
    dax = gyro(1) * dt;
    day = gyro(2) * dt;
    daz = gyro(3) * dt;
    d_angle_b = [dax; day; daz];

%% delta da velocidade pelo acelerometro
    dvx = acc(1) * dt;
    dvy = acc(2) * dt;
    dvz = acc(3) * dt;
    d_vel_b = [dvx; dvy; dvz];

%% quaternions k
    q0_k = x_k(1); 
    q1_k = x_k(2); 
    q2_k = x_k(3); 
    q3_k = x_k(4);

%% matriz de rotação de quaternions do eixo do corpo para o de navegação.
    rot_N_b = Q2R(q0_k, q1_k, q2_k, q3_k);
 
%% delta quaternion
    d_q0 = 1; % norma unitaria
    d_q1 = d_angle_b(1)/2;
    d_q2 = d_angle_b(2)/2;
    d_q3 = d_angle_b(3)/2;
   
%% quaternions k+1
    q0_k1 = (q0_k*d_q0) - (q1_k*d_q1) - (q2_k*d_q2) - (q3_k*d_q3);
    q1_k1 = (q0_k*d_q1) + (q1_k*d_q0) + (q2_k*d_q3) - (q3_k*d_q2);
    q2_k1 = (q0_k*d_q2) + (q2_k*d_q0) - (q1_k*d_q3) + (q3_k*d_q1);
    q3_k1 = (q0_k*d_q3) + (q3_k*d_q0) + (q1_k*d_q2) - (q2_k*d_q1);
   
    q_k1 = [q0_k1, q1_k1, q2_k1, q3_k1];
   
    q_k1_n = q_k1/sqrt(q_k1*q_k1'); %normalização do quaternion

%% velocidade k+1

    d_vel_N = rot_N_b * d_vel_b;
   
    vel_N_k = x_k(5); 
    vel_E_k = x_k(6); 
    vel_D_k = x_k(7); 
   
    vel_N_k1 = vel_N_k + dt*g(1) + d_vel_N(1);
    vel_E_k1 = vel_E_k + dt*g(2) + d_vel_N(2);
    vel_D_k1 = vel_D_k + dt*g(3) + d_vel_N(3);
   
%% posição k+1

    pos_N_k = x_k(8); 
    pos_E_k = x_k(9); 
    pos_D_k = x_k(10);

    pos_N_k1 = pos_N_k + dt*vel_N_k;
    pos_E_k1 = pos_E_k + dt*vel_E_k;
    pos_D_k1 = pos_D_k + dt*vel_D_k;

    x_k1_ = [q_k1_n(1); q_k1_n(2); q_k1_n(3); q_k1_n(4); vel_N_k1; vel_E_k1; vel_D_k1; pos_N_k1; pos_E_k1; pos_D_k1];
 
%% covariance

F = [                               1,                               -dax/2,                               -day/2,                               -daz/2,  0,  0,  0, 0, 0, 0;
                                dax/2,                                    1,                                daz/2,                               -day/2,  0,  0,  0, 0, 0, 0;
                                day/2,                               -daz/2,                                    1,                                dax/2,  0,  0,  0, 0, 0, 0;
                                daz/2,                                day/2,                               -dax/2,                                    1,  0,  0,  0, 0, 0, 0;
 2*dvx*q0_k - 2*dvy*q3_k + 2*dvz*q2_k, 2*dvx*q1_k + 2*dvy*q2_k + 2*dvz*q3_k, 2*dvy*q1_k - 2*dvx*q2_k + 2*dvz*q0_k, 2*dvz*q1_k - 2*dvy*q0_k - 2*dvx*q3_k,  1,  0,  0, 0, 0, 0;
 2*dvx*q3_k + 2*dvy*q0_k - 2*dvz*q1_k, 2*dvx*q2_k - 2*dvy*q1_k - 2*dvz*q0_k, 2*dvx*q1_k + 2*dvy*q2_k + 2*dvz*q3_k, 2*dvx*q0_k - 2*dvy*q3_k + 2*dvz*q2_k,  0,  1,  0, 0, 0, 0;
 2*dvy*q1_k - 2*dvx*q2_k + 2*dvz*q0_k, 2*dvx*q3_k + 2*dvy*q0_k - 2*dvz*q1_k, 2*dvy*q3_k - 2*dvx*q0_k - 2*dvz*q2_k, 2*dvx*q1_k + 2*dvy*q2_k + 2*dvz*q3_k,  0,  0,  1, 0, 0, 0;
                                    0,                                    0,                                    0,                                    0, dt,  0,  0, 1, 0, 0;
                                    0,                                    0,                                    0,                                    0,  0, dt,  0, 0, 1, 0;
                                    0,                                    0,                                    0,                                    0,  0,  0, dt, 0, 0, 1];
 
 
Q = [mrp_q   0     0     0     0     0     0     0     0     0
       0   mrp_q   0     0     0     0     0     0     0     0
       0     0   mrp_q   0     0     0     0     0     0     0
       0     0     0   mrp_q   0     0     0     0     0     0
       0     0     0     0   mrp_v   0     0     0     0     0
       0     0     0     0     0   mrp_v   0     0     0     0
       0     0     0     0     0     0   mrp_v   0     0     0
       0     0     0     0     0     0     0   mrp_p   0     0
       0     0     0     0     0     0     0     0   mrp_p   0
       0     0     0     0     0     0     0     0     0   mrp_p];

P_k1_ = F*P_k*F'+Q;
end
