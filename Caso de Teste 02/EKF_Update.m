function [x_k1, P_k1] = EKF_Update(acc_b, mag_b, x_k1_, P_k1_, std_acc, std_mag, g, h)

% h = @(x) [
% -2 * (x(2)   * x(4)   - x(1)   * x(3));
% -2 * (x(1)   * x(2)   + x(3)   * x(4));
% -    (x(1)^2 + x(4)^2 - x(2)^2 - x(3)^2);
%      (x(1)^2 + x(2)^2 - x(3)^2 - x(4)^2) * bx + 2 * (x(2)   * x(4) - x(1)*x(3))*bz;
%  2 * (x(2)   * x(3)   - x(1)   * x(4))   * bx + 2 * (x(1)   * x(2) - x(3)*x(4))*bz;
%  2 * (x(1)   * x(3)   + x(2)   * x(4))   * bx +     (x(1)^2 - x(2)^2 - x(3)^2+x(4)^2)*bz];
% 
% Hb1 =  x_k1_(1) * bx - x_k1_(3) * bz;
% Hb2 =  x_k1_(2) * bx + x_k1_(4) * bz;
% Hb3 = -x_k1_(3) * bx - x_k1_(1) * bz;
% Hb4 = -x_k1_(4) * bx + x_k1_(2) * bz;
% 
% H = @(x)[
%     2 * [x(3), -x(4),  x(1), -x(2), 0, 0, 0;
%          -x(2), -x(1), -x(4), -x(3), 0, 0, 0;
%          -x(1),  x(2),  x(3), -x(4), 0, 0, 0;
%          Hb1,    Hb2,    Hb3,    Hb4,    0, 0, 0;
%          Hb4,    -Hb3,   Hb2,    -Hb1,   0, 0, 0;
%         -Hb3,   -Hb4,   Hb1,    Hb2,    0, 0, 0;];
%         ];

g_n = g' ./ norm(g');          
gx = g_n(1);
gy = g_n(2);
gz = g_n(3);

rx = h(1);
ry = h(2);
rz = h(3);

h = @(x) 2 * [
    gx * (0.5 - x(3)^2 - x(4)^2) + gy * (x(1)*x(4) + x(2)*x(3)) + gz * (x(2)*x(4) - x(1)*x(3));
    gx * (x(2)*x(3) - x(1)*x(4)) + gy * (0.5 - x(2)^2 - x(4)^2) + gz * (x(1)*x(2) + x(3)*x(4));
    gx * (x(1)*x(3) + x(2)*x(4)) + gy * (x(3)*x(4) - x(1)*x(2)) + gz * (0.5 - x(2)^2 - x(3)^2);
    rx * (0.5 - x(3)^2 - x(4)^2) + ry * (x(1)*x(4) + x(2)*x(3)) + rz * (x(2)*x(4) - x(1)*x(3));
    rx * (x(2)*x(3) - x(1)*x(4)) + ry * (0.5 - x(2)^2 - x(4)^2) + rz * (x(1)*x(2) + x(3)*x(4));
    rx * (x(1)*x(3) + x(2)*x(4)) + ry * (x(3)*x(4) - x(1)*x(2)) + rz * (0.5 - x(2)^2 - x(3)^2)
];


H = @(x) 2 * [
    gx*x(1) + gy*x(4) - gz*x(3),  gx*x(2) + gy*x(3) + gz*x(4), -gx*x(3) + gy*x(2) - gz*x(1), -gx*x(4) + gy*x(1) + gz*x(2), 0, 0, 0;
   -gx*x(4) + gy*x(1) + gz*x(2),  gx*x(3) - gy*x(2) + gz*x(1),  gx*x(2) + gy*x(3) + gz*x(4), -gx*x(1) - gy*x(4) + gz*x(3), 0, 0, 0;
    gx*x(3) - gy*x(2) + gz*x(1),  gx*x(4) - gy*x(1) - gz*x(2),  gx*x(1) + gy*x(4) - gz*x(3),  gx*x(2) + gy*x(3) + gz*x(4), 0, 0, 0;
    rx*x(1) + ry*x(4) - rz*x(3),  rx*x(2) + ry*x(3) + rz*x(4), -rx*x(3) + ry*x(2) - rz*x(1), -rx*x(4) + ry*x(1) + rz*x(2), 0, 0, 0;
   -rx*x(4) + ry*x(1) + rz*x(2),  rx*x(3) - ry*x(2) + rz*x(1),  rx*x(2) + ry*x(3) + rz*x(4), -rx*x(1) - ry*x(4) + rz*x(3), 1, 0, 0;
    rx*x(3) - ry*x(2) + rz*x(1),  rx*x(4) - ry*x(1) - rz*x(2),  rx*x(1) + ry*x(4) - rz*x(3),  rx*x(2) + ry*x(3) + rz*x(4), 0, 1, 0
];


y_k1 = [acc_b(1); acc_b(2); acc_b(3); mag_b(1); mag_b(2); mag_b(3)];

R = diag([std_acc, std_acc, std_acc, std_mag, std_mag, std_mag]);

% Atualização
Hk = H(x_k1_);
K = P_k1_ * Hk' / (Hk * P_k1_ * Hk' + R);
x_k1 = x_k1_ + K * (y_k1 - h(x_k1_));
P_k1 = (eye(length(x_k1_)) - K * Hk) * P_k1_;

% Normalização do quaternion
x_k1(1:4) = x_k1(1:4) / norm(x_k1(1:4));
end
