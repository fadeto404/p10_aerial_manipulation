function T_inv = homogeneous_tf_inv(T)
    T_inv = [T(1:3,1:3)', -T(1:3,4); 0,0,0,1];
end