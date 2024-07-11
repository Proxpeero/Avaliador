function [E] = M2E(R)
roll = atan2(R(1,2),R(1,1));

if (R(1,3)^2 < 1.0)
    pitch = -atan2(R(1,3),sqrt(1-R(1,3)^2));
else
    pitch = -atan2(R(1,3),0);
end

yaw = atan2(R(2,3),R(3,3));

E = [roll, pitch, yaw];