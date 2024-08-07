function [E] = Q2E2(Q)
q0 = Q(1);
q1 = Q(2);
q2 = Q(3);
q3 = Q(4);

roll = atan2(2*(q0*q1 + q2*q3), 1 - 2*(q1^2 + q2^2));
pitch = -asin(2*(q0*q2 - q3*q1));
yaw = -atan2(2*(q1*q2 + q0*q3), q0^2 + q1^2 - q2^2 - q3^2);

E = [roll, pitch, yaw];