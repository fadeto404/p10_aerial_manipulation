function [tau, f, n_hat] = recursive_newton_euler_world(q, q_dot, q_ddot, m, r, I, e)
% Recursive Newton-Euler algorithm for a robotic manipulator with revolute joints
% Inputs:
%   q:      n x 1 vector of joint angles
%   q_dot:  n x 1 vector of joint velocities
%   q_ddot: n x 1 vector of joint accelerations
%   m:      1 x n vector of link masses
%   r:      3 x (n+1) matrix of base position and link COM positions in the world frame
%   I:      3 x 3 x n array of link inertias in the world frame
%   e:      3 x n matrix of joint axes in the world frame
% Outputs:
%   tau:    n x 1 vector of joint torques
%   f:      3 x n matrix of link forces
%   n_hat:  3 x n matrix of link torques

% TODO: 
n = length(q);              % number of joints
f = sym(zeros(3, n+1));     % forces acting on each link
F = sym(zeros(3, n+1));     % forces acting on each link
n_hat = sym(zeros(3, n+1)); % moments acting on each link
N_hat = sym(zeros(3, n+1)); % moments acting on each link
tau = sym(zeros(n, 1));     % joint torques

% Forward pass
for i = 1:n
    % Calculate joint velocity and acceleration
    omega_i = e(:, i) * q_dot(i);
    alpha_i = e(:, i) * q_ddot(i) + cross(omega_i, e(:, i) * q_dot(i));
    
    % Calculate linear and angular acceleration of the link
    a_i = F(:, i) / m(i);
    Delta_r_i = r(:, i+1) - r(:, i);
    alpha_omega_Delta_r_i = cross(alpha_i, Delta_r_i) + cross(omega_i, cross(omega_i, Delta_r_i));
    
    % Calculate force and moment acting on the link
    F(:, i+1) = m(i) * (a_i + alpha_omega_Delta_r_i);
    N_hat(:, i+1) = I(:, :, i) * alpha_i + cross(omega_i, I(:, :, i) * omega_i);
end

% Backward pass
for i = n:-1:1
    % Calculate joint velocity and acceleration
    omega_i = e(:, i) * q_dot(i);
    alpha_i = e(:, i) * q_ddot(i) + cross(omega_i, e(:, i) * q_dot(i));
    
    % Calculate torque acting on the joint
    n_hat_i = N_hat(:, i+1) + cross(r(:, i+1) - r(:, i), f(:, i+1));
    tau(i) = transpose(e(:, i)) * n_hat_i;
    
    % Pass forces and moments to parent link
    f(:, i) = f(:, i+1);
    n_hat(:, i) = n_hat_i;
end