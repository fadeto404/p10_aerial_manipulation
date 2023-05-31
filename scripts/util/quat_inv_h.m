% Quaternion inversion, q must be 4x1, q(2:4) vector part, q(1) scalar part
function q_inv = quat_inv_h(q)
    q_inv = [q(1); -q(2:4)]./norm(q);
end