function [x_k1_,P_k1_] = EKF_Prediction(gyro, dt, x_k, P_k, mrp_q)
%% Saídas

   % quaternion = q0, q1, q2, q3;
   
%% delta do ângulo calculado pelo giroscopio
    dax = gyro(1) * dt;
    day = gyro(2) * dt;
    daz = gyro(3) * dt;
    d_angle_b = [dax; day; daz];

%% quaternions k
    q0_k = x_k(1); 
    q1_k = x_k(2); 
    q2_k = x_k(3); 
    q3_k = x_k(4);
 
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

    x_k1_ = [q_k1_n(1); q_k1_n(2); q_k1_n(3); q_k1_n(4)];
 
%% covariance

F = [    1   -dax/2   -day/2   -daz/2;
     dax/2        1    daz/2   -day/2;
     day/2   -daz/2        1    dax/2;
     daz/2    day/2   -dax/2        1];
 
w_process_noise = 1e-6 * ones(1, 4);
w_bias_process_noise = 1e-8 * ones(1,3);
Q = diag([w_process_noise, w_bias_process_noise]);

% Q = [mrp_q   0     0     0     
%        0   mrp_q   0     0     
%        0     0   mrp_q   0     
%        0     0     0   mrp_q];

P_k1_ = F*P_k*F'+Q;
end
