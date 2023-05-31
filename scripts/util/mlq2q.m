function q = mlq2q(mlq)
%mlq2q: Matlab style quaternion to my custom quaternion format
% Inputs: 
%   mlq:    n x 4 array of quaternions arranged as [w x y z]
% Outputs:
%   q:      4 x n array of quaternions arranged as [x; y; z; w]
    q(1:3,:) = mlq(:,2:4)';
    q(4,:) = mlq(:,1)';
end