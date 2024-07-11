function [x_k1, P_k1] = atualizacao(acc_b, mag_b, heading, vel_GPS, pos_GPS, g, x_k1_, P_k1_, mcm_a, mcm_m, mcm_v, mcm_p)

q0_k = x_k1_(1);
q1_k = x_k1_(2);
q2_k = x_k1_(3);
q3_k = x_k1_(4);

y_k1_q = [acc_b(1); acc_b(2); acc_b(3); mag_b(1); mag_b(2); mag_b(3)];

R_q = [ mcm_a^2    0       0       0       0       0
           0    mcm_a^2    0       0       0       0
           0       0    mcm_a^2    0       0       0
           0       0       0    mcm_m^2    0       0
           0       0       0       0    mcm_m^2    0
           0       0       0       0       0    mcm_m^2];
 
 g_n = g' ./ norm(g');          
 gx = g_n(1);
 gy = g_n(2);
 gz = g_n(3);
 
 rx = heading(1);
 ry = heading(2);
 rz = heading(3);

h_k1_q = 2* [gx * (0.5 - (q2_k)^2 - (q3_k)^2) + gy * (q0_k * q3_k + q1_k * q2_k) + gz * (q1_k * q3_k - q0_k * q2_k);
             gx * (q1_k * q2_k - q0_k * q3_k) + gy * (0.5 - (q1_k)^2 - (q3_k)^2) + gz * (q0_k * q1_k + q2_k * q3_k);
             gx * (q0_k * q2_k + q1_k * q3_k) + gy * (q2_k * q3_k - q0_k * q1_k) + gz * (0.5 - (q1_k)^2 - (q2_k)^2);
             rx * (0.5 - (q2_k)^2 - (q3_k)^2) + ry * (q0_k * q3_k + q1_k * q2_k) + rz * (q1_k * q3_k - q0_k * q2_k);
             rx * (q1_k * q2_k - q0_k * q3_k) + ry * (0.5 - (q1_k)^2 - (q3_k)^2) + rz * (q0_k * q1_k + q2_k * q3_k);
             rx * (q0_k * q2_k + q1_k * q3_k) + ry * (q2_k * q3_k - q0_k * q1_k) + rz * (0.5 - (q1_k)^2 - (q2_k)^2)];
   
H_q = [ 2*gy*q3_k - 2*gz*q2_k,              2*gy*q2_k + 2*gz*q3_k,  2*gy*q1_k - 4*gx*q2_k - 2*gz*q0_k,  2*gy*q0_k - 4*gx*q3_k + 2*gz*q1_k;
        2*gz*q1_k - 2*gx*q3_k,  2*gx*q2_k - 4*gy*q1_k + 2*gz*q0_k,              2*gx*q1_k + 2*gz*q3_k,  2*gz*q2_k - 4*gy*q3_k - 2*gx*q0_k;
        2*gx*q2_k - 2*gy*q1_k,  2*gx*q3_k - 2*gy*q0_k - 4*gz*q1_k,  2*gx*q0_k + 2*gy*q3_k - 4*gz*q2_k,              2*gx*q1_k + 2*gy*q2_k;
        2*q3_k*ry - 2*q2_k*rz,              2*q2_k*ry + 2*q3_k*rz,  2*q1_k*ry - 4*q2_k*rx - 2*q0_k*rz,  2*q0_k*ry - 4*q3_k*rx + 2*q1_k*rz;
        2*q1_k*rz - 2*q3_k*rx,  2*q2_k*rx - 4*q1_k*ry + 2*q0_k*rz,              2*q1_k*rx + 2*q3_k*rz,  2*q2_k*rz - 4*q3_k*ry - 2*q0_k*rx;
        2*q2_k*rx - 2*q1_k*ry,  2*q3_k*rx - 2*q0_k*ry - 4*q1_k*rz,  2*q0_k*rx + 2*q3_k*ry - 4*q2_k*rz,              2*q1_k*rx + 2*q2_k*ry];

P_k1_q_ = [P_k1_(1,1:4); P_k1_(2,1:4); P_k1_(3,1:4); P_k1_(4,1:4)];
x_k1_q_ = [x_k1_(1); x_k1_(2); x_k1_(2); x_k1_(4)];

S_q = H_q * P_k1_q_ * H_q' + R_q;
P_q = P_k1_q_ * H_q';
K_k_q  =  P_q / S_q;

KH_q = K_k_q * H_q;
I_q = eye(size(KH_q));
I_KH_q = I_q - KH_q;
P_k1_q = I_KH_q * P_k1_q_ * I_KH_q' + K_k_q * R_q * K_k_q';

Z_q = y_k1_q - h_k1_q;
x_k1_q = x_k1_q_ + K_k_q * Z_q;

%% Velocidade

R_v = [mcm_v^2    0       0           
          0    mcm_v^2    0          
          0       0    mcm_v^2];

H_v = [1   0   0           
       0   1   0          
       0   0   1];
   
y_k1_v = [vel_GPS(1)
          vel_GPS(2)
          vel_GPS(3)];

h_k1_v = [x_k1_(5)
          x_k1_(6)
          x_k1_(7)];

P_k1_v_ = [P_k1_(5,5:7); P_k1_(6,5:7); P_k1_(7,5:7)];
x_k1_v_ = [x_k1_(5); x_k1_(6); x_k1_(7)];

S_v = H_v * P_k1_v_ * H_v' + R_v;
P_v = P_k1_v_ * H_v';
K_k_v  =  P_v / S_v;

KH_v = K_k_v * H_v;
I_v = eye(size(KH_v));
I_KH_v = I_v - KH_v;
P_k1_v = I_KH_v * P_k1_v_ * I_KH_v' + K_k_v * R_v * K_k_v';

Z_v = y_k1_v - h_k1_v;
x_k1_v = x_k1_v_ + K_k_v * Z_v;

%% Posição  

R_p = [mcm_p^2    0       0           
          0    mcm_p^2    0          
          0       0    mcm_p^2];

H_p = [1   0   0           
       0   1   0          
       0   0   1];
   
y_k1_p = [pos_GPS(1)
          pos_GPS(2)
          pos_GPS(3)];

h_k1_p = [x_k1_(8)
          x_k1_(9)
          x_k1_(10)];


P_k1_p_ = [P_k1_(8,8:10); P_k1_(9,8:10); P_k1_(10,8:10)];
x_k1_p_ = [x_k1_(8); x_k1_(9); x_k1_(10)];

S_p = H_p * P_k1_p_ * H_p' + R_p;
P_p = P_k1_p_ * H_p';
K_k_p  =  P_p / S_p;

KH_p = K_k_p * H_p;
I_p = eye(size(KH_p));
I_KH_p = I_p - KH_p;
P_k1_p = I_KH_p * P_k1_p_ * I_KH_p' + K_k_p * R_p * K_k_p';

Z_p = y_k1_p - h_k1_p;
x_k1_p = x_k1_p_ + K_k_p * Z_p; 

%% Saída 
P_k1 = [P_k1_q(1,1)     P_k1_q(1,2)     P_k1_q(1,3)     P_k1_q(1,4)               0               0               0               0               0                 0
        P_k1_q(2,1)     P_k1_q(2,2)     P_k1_q(2,3)     P_k1_q(2,4)               0               0               0               0               0                 0
        P_k1_q(3,1)     P_k1_q(3,2)     P_k1_q(3,3)     P_k1_q(3,4)               0               0               0               0               0                 0
        P_k1_q(4,1)     P_k1_q(4,2)     P_k1_q(4,3)     P_k1_q(4,4)               0               0               0               0               0                 0
                  0               0               0               0     P_k1_v(1,1)     P_k1_v(1,2)     P_k1_v(1,3)               0               0                 0
                  0               0               0               0     P_k1_v(2,1)     P_k1_v(2,2)     P_k1_v(2,3)               0               0                 0
                  0               0               0               0     P_k1_v(3,1)     P_k1_v(3,2)     P_k1_v(3,3)               0               0                 0
                  0               0               0               0               0               0               0     P_k1_p(1,1)     P_k1_p(1,2)      P_k1_p(1,3)
                  0               0               0               0               0               0               0     P_k1_p(2,1)     P_k1_p(2,2)      P_k1_p(2,3)
                  0               0               0               0               0               0               0     P_k1_p(3,1)     P_k1_p(3,2)      P_k1_p(3,3)];
    
x_k1 = [x_k1_q; x_k1_v; x_k1_p];
end
