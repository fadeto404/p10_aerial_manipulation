% Conversion from my custom quaternion to matlab quaternion
function mlq = q2mlq(q)
    mlq(:,1) = q(4,:)';
    mlq(:,2:4) = q(1:3,:)';
end