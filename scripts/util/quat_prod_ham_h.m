function q_new = quat_prod_ham_h(q1, q2)
    if length(q2) == 4
        q_new = [q1(1)*q2(1) - transpose(q1(2:4))*q2(2:4); q2(1)*q1(2:4) + q1(1)*q2(2:4) + cross(q1(2:4), q2(2:4))];
    elseif length(q2) == 3
        q_new = [-transpose(q1(2:4))*q2; q1(1)*q2 + cross(q1(2:4), q2)];
    end
end