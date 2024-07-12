function [Q] = E2Q(roll, pitch, yaw)
	cYaw = cos(yaw / 2);
    sYaw = sin(yaw / 2);
	cPit = cos(pitch / 2);
    sPit = sin(pitch / 2);
	cRol = cos(roll / 2);
    sRol = sin(roll / 2);

	q0 = cRol*cPit*cYaw + sRol*sPit*sYaw;
	q1 = sRol*cPit*cYaw - cRol*sPit*sYaw;
	q2 = cRol*sPit*cYaw + sRol*cPit*sYaw;
	q3 = cRol*cPit*sYaw - sRol*sPit*cYaw;
    
    Q = [q0, q1, q2, q3];
end