function [E] = M2E(R)
yaw = atan2(R(3,2), R(3,3));

pitch = asin(-R(3,1));

roll = atan2(R(2,1), -R(1,1));

E = [roll, pitch, yaw];