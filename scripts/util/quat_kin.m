function dq = quat_kin(q, omega)
% Quaternion kinematic function
    dq = 0.5*quat_prod_ham(q, omega);
end
