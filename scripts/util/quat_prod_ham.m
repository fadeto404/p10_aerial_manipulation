function q_new = quat_prod_ham(q1, q2)
    if length(q2) == 4
        q_new = [q2(4)*q1(1:3) + q1(4)*q2(1:3) + cross(q1(1:3), q2(1:3)); q1(4)*q2(4) - transpose(q1(1:3))*q2(1:3)];
    elseif length(q2) == 3
        q_new = [q1(4)*q2 + cross(q1(1:3), q2); -transpose(q1(1:3))*q2];
    end
end