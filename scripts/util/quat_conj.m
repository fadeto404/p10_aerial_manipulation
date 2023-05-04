function q_conj = quat_conj(q)
    q_conj = [-q(1:3); q(4)];
end