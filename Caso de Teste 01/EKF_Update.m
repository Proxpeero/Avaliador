function [x_k1, P_k1] = EKF_Update(acc_b, mag_b, heading, g, x_k1_, P_k1_, mcm_a, mcm_m)

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
x_k1 = x_k1_q_ + K_k_q * Z_q;

%% Saída 
P_k1 = [P_k1_q(1,1)   P_k1_q(1,2)   P_k1_q(1,3)   P_k1_q(1,4)
        P_k1_q(2,1)   P_k1_q(2,2)   P_k1_q(2,3)   P_k1_q(2,4)
        P_k1_q(3,1)   P_k1_q(3,2)   P_k1_q(3,3)   P_k1_q(3,4)  
        P_k1_q(4,1)   P_k1_q(4,2)   P_k1_q(4,3)   P_k1_q(4,4)];
end
