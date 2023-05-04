function q_R = R2q(R)
    q_temp = rotm2quat(R);
    q_R = [q_temp(2:4)'; q_temp(1)];
end