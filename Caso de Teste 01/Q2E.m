function [E] = Q2E(Q)
q0 = Q(1);
q1 = Q(2);
q2 = Q(3);
q3 = Q(4);

M11 = q0^2+q1^2-q2^2-q3^2;
M12 = 2*(q1*q2+q0*q3);
M13 = 2*(q1*q3-q0*q2);
M21 = 2*(q1*q2-q0*q3);
M22 = q0^2-q1^2+q2^2-q3^2;
M23 = 2*(q2*q3+q0*q1);
M31 = 2*(q1*q3+q0*q2);
M32 = 2*(q2*q3-q0*q1);
M33 = q0^2-q1^2-q2^2+q3^2;

M = [M11 M12 M13
     M21 M22 M23
     M31 M32 M33];

roll = atan2(M(2,3),M(3,3));

if (M(1,3)^2 < 1.0)
    pitch = -atan2(M(1,3),sqrt(1-M(1,3)^2));
else
    pitch = -atan2(M(1,3),0);
end

yaw = atan2(M(1,2),M(1,1));

E = [roll, pitch, yaw];