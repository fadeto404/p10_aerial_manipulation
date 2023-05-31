function [tau, f, n_hat] = recursive_newton_euler(q, q_dot, q_ddot, m, r, p, I, R, om0, dom0, v0, dv0)
% Recursive Newton-Euler algorithm for a robotic manipulator with revolute joints.
% Refer to:
% "Introduction to Robotics: Mechanics and Control", by John J. Craig, 
% 3rd ed. (2005).
% Inputs:
%   q:      n x 1 vector of joint angles
%   q_dot:  n x 1 vector of joint velocities
%   q_ddot: n x 1 vector of joint accelerations
%   m:      1 x n vector of link masses
%   r:      3 x (n+1) matrix of base position and link COM positions in the link frame
%   p:      3 x (n+1) matrix of joint positions in link frame and last is ee frame
%   I:      3 x 3 x n array of link inertias in the CoM frame (displaced link frame)
%   R:      3 x 3 x n array of rotation matrices from base to link i
%   om0:    3 x 1 vector of base frame angular velocity
%   dom0:   3 x 1 vector of angular acceleration of base frame
%   v0:     3 x 1 vector of velocity of base frame
%   a0:     3 x 1 vector of acceleration of base frame (add gravity here)
% Outputs:
%   tau:    n x 1 vector of joint torques
%   f:      3 x n matrix of link forces
%   n_hat:  3 x n matrix of link torques

% TODO: use DH parameters for frame pos and rotation matrices
n = length(q);              % number of joints
f = sym(zeros(3, n+1));     % forces exerted on link i by link i-1
F = sym(zeros(3, n+1));     % forces acting on each link
n_hat = sym(zeros(3, n+1)); % torques exerted on link i by link i-1
N_hat = sym(zeros(3, n+1)); % torques acting on each link
tau = sym(zeros(n, 1));     % joint torques
om(:,1) = om0;              % angular velocity of base frame
dom(:,1) = dom0;            % angular acceleration of base frame
v(:,1) = v0;                % velocity of base frame
dv(:,1) = dv0;              % acceleration of base frame
dv_c = sym(zeros(3,n));     % acceleration of CoM of each link
z = [0;0;1];                % joint axis in local frame (z-vector)

% ===============================
% matlab i = algorithm i+1
% ===============================
% om(:,i) = i+1(w)_i+1 
% om(:,i-1) = i(w)_i
% dom(:,i) = i+1(dot w)_i+1 
% v(:,i) = ?
% dv(:,i) = i+1(dot v)_i+1
% R(:,:,i-1) = ^i_{i+1}(R)
% transpose(R(:,:,i-1)) = ^{i+1}_i(R)
% p(:,i) = i(p)_i+1
% ===============================
% matlab i-1 = algorithm i+1
% ===============================
% q_dot(i-1) = dtheta_i+1
% q_ddot(i-1) = ddtheta_i+1
% dv_c(:,i-1) = i+1(dot v)_C_i+1
% r(:,i-1) = i+1(p)_C_i+1
%
% z = i+1(z)_i+1 = i(z)_i

% Outward iteration/forward pass (not to be confused with forward dynamics)
for i=2:n+1
    % Angular velocity of link
    om(:,i) = simplify(transpose(R(:,:,i-1)) * om(:,i-1) + q_dot(i-1)*z);
    
    % Angular acceleration of link
    dom(:,i) = simplify(transpose(R(:,:,i-1)) * dom(:,i-1) + q_ddot(i-1)*z + ...
               cross(transpose(R(:,:,i-1)) *om(:,i-1), q_dot(i-1)*z));
    
    % Velocity of link frame
    % v(:, i) = R(:,:,i-1) * v(:,i-1) + cross(om(:,i), p(:,i-1))

    % Acceleration of link frame
    dv(:, i) = simplify(transpose(R(:,:,i-1)) * (dv(:,i-1) + ...
               cross(dom(:,i-1), p(:,i-1)) + ...
               cross(om(:,i-1), cross(om(:,i-1), p(:,i-1)))));
    
    % Acceleration of link CoM
    dv_c(:,i-1)= simplify(cross(dom(:,i), r(:,i-1)) + ...
                 cross(om(:,i), cross(om(:,i), r(:,i-1))) + ...
                 dv(:,i));

    % Force acting on link CoM
    F(:,i-1) = simplify(m(i-1) * dv_c(:,i-1));

    % Torque acting on link CoM
    N_hat(:,i-1) = simplify(I(:,:,i-1) * dom(:,i) + ...
                   cross(dom(:,i), I(:,:,i-1) * dom(:,i)));
end

% Inward iteration/backward pass
for i=n:-1:1
    f(:,i) = simplify(R(:,:,i+1) * f(:,i+1) + F(:,i));

    n_hat(:,i) = simplify(N_hat(:,i) + R(:,:,i+1) * n_hat(:,i+1) + ...
                 cross(r(:,i), F(:,i)) + ...
                 cross(p(:,i+1), (R(:,:,i+1)) * f(:,i+1)));

    % Joint torque
    tau(i) = simplify(transpose(n_hat(:,i)) * z);
end