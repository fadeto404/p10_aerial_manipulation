function R_q = q2R(q)
    R_q = (q(4)^2 + q(1:3)'*q(1:3))*eye(3) - 2*q(4)*cross_prod_mat(q) + 2*q(1:3)*q(1:3)';
end