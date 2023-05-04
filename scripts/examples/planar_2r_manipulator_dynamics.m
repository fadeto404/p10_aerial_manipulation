%% 2R-planar manipulator dynamics with recursive Newton-Euler
% Made for verifying the RNE implementation
clear; clc; close all;
%% Setup
% Parameters and variables
l_sym = sym('l',[1,2]);
m_sym = sym('m',[1,2]);
g_sym = sym('g');
% J_sym = sym('J',[3,3,2]);
J_sym = sym(zeros(3,3,2));
f_sym = transpose(sym('f',[3,3]));
f_sym(:,3) = sym(zeros(3,1));
n_sym = transpose(sym('n', [3,3]));
n_sym(:,3) = sym(zeros(3,1));
om0 = sym(zeros(3,1));
dom0 = sym(zeros(3,1));
v0 = sym(zeros(3,1));
dv0 = sym([0; g_sym; 0]);
theta_sym = sym('th', [1,2]);
dtheta_sym = sym('dth', [1,2]);
ddtheta_sym = sym('ddth', [1,2]);
R01 = [cos(theta_sym(1)), -sin(theta_sym(1)), 0;
       sin(theta_sym(1)), cos(theta_sym(1)), 0;
       0, 0, 1];
R12 = [cos(theta_sym(2)), -sin(theta_sym(2)), 0;
       sin(theta_sym(2)), cos(theta_sym(2)), 0;
       0, 0, 1];
R_array(:,:,1) = R01;
R_array(:,:,2) = R12;
R_array(:,:,3) = eye(3);
P_c = [[l_sym(1), l_sym(2)]; zeros(2,2)];
P_i = [zeros(3,1), P_c];

[jt, fl, nl] = recursive_newton_euler(theta_sym, dtheta_sym, ddtheta_sym, m_sym, P_c, P_i, J_sym, R_array, om0, dom0, v0, dv0)

% Reference from book (Craig, 2005)
ref_jt1 = m_sym(2)*(l_sym(2)^2)*(ddtheta_sym(1)+ddtheta_sym(2)) + ...
         m_sym(2) * l_sym(1) * l_sym(2) * cos(theta_sym(2)) * (2* ddtheta_sym(1) + ddtheta_sym(2)) + ...
         (m_sym(1) + m_sym(2))*(l_sym(1)^2)*ddtheta_sym(1) - ...
         m_sym(2) * l_sym(1) * l_sym(2) * sin(theta_sym(2)) * dtheta_sym(2)^2 - ...
         2 * m_sym(2) * l_sym(1) * l_sym(2) * sin(theta_sym(2)) * dtheta_sym(1) * dtheta_sym(2) + ...
         m_sym(2) * l_sym(2) * g_sym * cos(theta_sym(1) + theta_sym(2)) + ...
         (m_sym(1) + m_sym(2)) * l_sym(1) * g_sym * cos(theta_sym(1))

ref_func = matlabFunction(ref_jt1);
tfunc = matlabFunction(jt(1));
%% check equivalence
ref_func(1,1,0.5,0.5,9.82,1,1,1,1,pi/2,pi/2)
tfunc(1,1,0.5,0.5,9.82,1,1,1,1,pi/2,pi/2)












