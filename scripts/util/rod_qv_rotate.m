%% Rotate vector by unit quaternion using Rodrigues' formula
function v_new = rod_qv_rotate(quaternion, vector)
    % v_new = vector + 2*cross(quaternion(1:3), cross(quaternion(1:3), vector) + quaternion(4)*vector);
    temp = [quaternion(2)*vector(3) - quaternion(3)*vector(2); quaternion(3)*vector(1) - quaternion(1)*vector(3); quaternion(1)*vector(2) - quaternion(2)*vector(1)] + quaternion(4)*vector;
    v_new = vector + 2*[quaternion(2)*temp(3) - quaternion(3)*temp(2); quaternion(3)*temp(1) - quaternion(1)*temp(3); quaternion(1)*temp(2) - quaternion(2)*temp(1)];
end