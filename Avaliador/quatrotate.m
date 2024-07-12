function v = quatrotate(q, r)
    % Rotaciona um vetor r pelo quaternion q
    qConj = [q(1) -q(2) -q(3) -q(4)];
    v = quatmultiply(quatmultiply(q, [0 r]), qConj);
    v = v(2:4); % Retorna apenas a parte vetorial
end