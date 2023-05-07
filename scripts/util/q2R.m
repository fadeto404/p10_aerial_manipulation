function R_q = q2R(q)
% q2R: Compute equivalent rotation matrix from quaternion.
% Inputs:
%   q:      4 x 1 vector/quaternion [x; y; z; w] (or [q1; q2; q3; q4])
% Outputs:
%   R_q:    3 x 3 rotation matrix equivalent to q
    R_q = (q(4)^2 - transpose(q(1:3))*q(1:3))*eye(3) - 2*q(4)*cross_prod_mat(q(1:3)) + 2*q(1:3)*transpose(q(1:3));
end