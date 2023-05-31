clear; clc; close all;
%% Config
run util/plot_settings.m
N_t = 1500; % No. of time samples
t_s = 0.005; % Time step size [s]
T_sim = 0:t_s:t_s*(N_t-1); % Time vector for simulation

%% Quadrotor parameters
% Parameters
m0 = 1; % Base mass [kg]
J0 = eye(3); %[1, -0.1, -0.1; -0.1, 1, -0.1; -0.1, -0.1, 1]; % Base inertia [?]

k_t = 1; % Thrust constant motor [kg/%]
k_tau = 0.01; % Rotor drag coefficient [???]
g = 9.816; % Gravitational acceleration at Earth [m/s^2]

% Vectors from CoM to each motor in body frame [m]
s1 = [ 0.2;  0.2;  -0.1];
s2 = [-0.2;  0.2;  -0.1];
s3 = [-0.2; -0.2;  -0.1];
s4 = [ 0.2; -0.2;  -0.1];
S = [s1, s2, s3, s4];
% S_sym = sym('s', [3, 4]);
% S = S_sym

%% Manipulator parameters
n_l = 3; % No. of links
n_j = 2; % No. of joints
n_b = 3; % No. of bodies (including B0)

% Define joint angles and link lengths
syms theta1 theta2 real 
% syms l0 l1 l2 real
l0 = 0.1; % Distance from base to joint 1 [m]
l1 = 0.05; % Distance from joint 1 to joint 2 [m]
l2 = 0.1; % Distance from joint 2 to tool [m]

% Link masses and inertia tensors
m1 = 0.2; % Link 1 mass [kg]
m2 = 0.1; % Link 2 mass [kg]
mp = 0.2; % Payload mass [kg]

% Inertia tensor of link at CoM in coordinate frame aligned with link frame
J1 = 0.2*eye(3); % Link 1 inertia [kg.m^2]
J2 = 0.1*eye(3); % Link 2 inertia [kg.m^2]
Jp = eye(3); % Payload inertia [kg.m^2]

P_ci = [[0;0;0], [0; 0; l1-0.01], [0; 0.1; 0]]; % CoM of link i in link i frame
P_i = [[0; 0; l0], [0; 0; l1], [0; -l2; 0]] ; % ^iP_i+1 Location of link i+1 frame as seen in frame i

% Torque constant
k_i = 1; % [Nm/A]

% Denavit-Hartenberg parameters
i = [1; 2; 3]; % Frame indices (probably unnecessary)
a_i = [0; 0; 0]; % Distance from Z_i to Z_(i+1) along X_i [m]
alpha_i = [0; -pi/2; pi/2]; % Angle from Z_i to Z_(i+1) around X_i [rad]
d_i = [l0+l1; 0; l2]; % Distance from X_(i-1) to X_i along Z_i [m]
theta_i = [theta1; theta2; 0]; % Angle from X_(i-1) to X_i around Z_i [rad]
dh_table = table(i, a_i, alpha_i, d_i, theta_i, 'VariableNames', {'i', 'a_{i-1}', 'alpha_{i-1}', 'd_{i}', 'theta_{i}'});
[T_0t_sym, T_i_sym] = dh_forward_kinematics(dh_table);

%% UAM parameters
% Vector of all body masses
m_vec = [m0;m1;m2]; 

% All inertia tensors in local body frame
Phi(:,:,1) = J0;
Phi(:,:,2) = J1;
Phi(:,:,3) = J2;

%% Kinematics of quadrotor
% Generalized coords
syms x_B y_B z_B x_B_dot y_B_dot z_B_dot w_x w_y w_z dtheta1 dtheta2 real
syms q0 q1 q2 q3 q4 dq0 dq1 dq2 dq3 dq4 real
dtheta_i = [dtheta1; dtheta2];

% Attitude quaternion (expressed in world frame)
%q_sym = [q1; q2; q3; q4];
%dq_sym = [dq1; dq2; dq3; dq4];
q_sym = [q0; q1; q2; q3];
dq_sym = [dq0; dq1; dq2; dq3];

% Attitude matrix related to quaternion q
%w_R_L0 = q2R(q_sym);
w_R_L0 = q2R([q_sym(2:4); q_sym(1)]);

% Position of link 0 relative to world frame in world frame
w_P_wL0 = [x_B; y_B; z_B];

% Velocity of link 0 relative to world frame in world frame
w_dP_wL0 = [x_B_dot; y_B_dot; z_B_dot];

% Homogeneous transformation matrix from world frame to link 0
w_A_L0 = [w_R_L0, w_P_wL0;
0, 0, 0, 1];

% 3D ang vel of L0 relative to W in W
w_Ombar_wL0 = w_R_L0*[w_x; w_y; w_z];

% G matrix
%G_mat = [q_sym(4)*eye(3) - cross_prod_mat(q_sym(1:3)), -q_sym(1:3)];
G_mat = [-q_sym(2:4), q_sym(1)*eye(3) - cross_prod_mat(q_sym(2:4))];

%% Kinematics of manipulator
% Homogeneous transformation from joint i to link i
for i=1:n_j
    Ji_A_Li(:,:,i) = [cos(theta_i(i)), -sin(theta_i(i)), 0, 0;
                      sin(theta_i(i)), cos(theta_i(i)) , 0, 0;
                      0              , 0               , 1, 0;
                      0              , 0               , 0, 1];
end

% Translation vectors from joint to link frames
for i=1:n_j
    Ji_P_JiLi(:,:,i) = Ji_A_Li(1:3,4,i);
end

% Position of body i relative to link i as seen from link i
for i=1:n_j
    Li_P_LiBi(:,:,i) = P_ci(:,i+1);
end

% Homogeneous transformation from joint i to body i
for i=1:n_j
    Ji_A_Bi(:,:,i) = Ji_A_Li(:,:,i)*[eye(3), Li_P_LiBi(:,:,i);
                                     0, 0, 0, 1];
end

% Position of joint 1 relative to link 0 in link 0 frame
L0_P_L0J1 = [0;0;l0];
L1_P_L1J2 = [0;0;l1];

% Homogeneous transformation from link 0 to joint 1 (constant)
L0_A_J1 = [eye(3), L0_P_L0J1;
0, 0, 0, 1];

% Homogeneous transformation from link 1 to joint 2 (constant)
L1_A_J2 = round([ext_rot_mat_xyz(pi/2,0,0), L1_P_L1J2;
0, 0, 0, 1], 5);

% Homogeneous transformation from link i to joint i+1 (constants)
Lim1_A_Ji(:,:,1) = L0_A_J1;
Lim1_A_Ji(:,:,2) = L1_A_J2;

% Homogeneous transformation from link 0 to joint i
L0_A_Ji(:,:,1) = sym(Lim1_A_Ji(:,:,1));
L0_A_Ji(:,:,2) = L0_A_Ji(:,:,1)*Ji_A_Li(:,:,1)*Lim1_A_Ji(:,:,2);

% Homogeneous transformation from inertial frame to joint i
for i=1:n_j
    w_A_Ji(:,:,i) = w_A_L0*L0_A_Ji(:,:,i);
end

% Homogeneous transformation from inertial frame to link i
for i=1:n_j
    w_A_Li(:,:,i) = w_A_Ji(:,:,i)*Ji_A_Li(:,:,i);
end

% Angular velocity of body i relative to joint i, as seen in joint i frame
for i=1:n_j
    Ji_Ombar_JiBi(:,:,i) = [0; 0; 1]*dtheta_i(i);
end

% Angular velocity of body i relative to joint i, as seen in link 0 frame
for i=1:n_j
    L0_Ombar_JiBi(:,:,i) = L0_A_Ji(1:3,1:3,i)*Ji_Ombar_JiBi(:,:,i);
end

% Angular velocity of body i relative to link 0, as seen in link 0 frame
L0_Ombar_L0Bi(:,:,1) = L0_Ombar_JiBi(:,:,1);
for i=2:n_j
    L0_Ombar_L0Bi(:,:,i) = L0_Ombar_JiBi(:,:,i)+L0_Ombar_L0Bi(:,:,i-1);
end

% Jacobian of rotation of body i in link 0 frame
for i=1:n_j
    L0_Jom_i(:,:,i) = jacobian(L0_Ombar_L0Bi(:,:,i), dtheta_i);
end
L0_Jom_0 = zeros(3,n_j);

% Position of joint i relative to link 0 as seen in link 0 frame
for i=1:n_j
    L0_P_L0Ji(:,:,i) = L0_A_Ji(1:3,4,i);
end

% Homogeneous transform of link i relative to link 0 as seen in link 0
% frame
for i=n_j
    % TODO? or unnecessary
end

% Homogeneous transform of body i relative to link 0 as seen in link 0 
% frame
for i=1:n_j
    L0_A_Bi(:,:,i) = L0_A_Ji(:,:,i)*Ji_A_Bi(:,:,i);
end

% Position of body i relative to link 0 as seen in link 0 frame
for i=1:n_j
    L0_P_L0Bi(:,:,i) = L0_A_Bi(1:3,4,i);
end

% Position of body i relative to joint j as seen in link 0 frame
L0_P_JjBi = sym(zeros(3,1,n_j,n_j));
for i=1:n_j
    for j=1:n_j
        L0_P_JjBi(:,:,i,j) = L0_P_L0Ji(:,:,j) - L0_P_L0Bi(:,:,i);
    end
end

% Joint axis j as seen in link 0
for j=1:n_j
    L0_n_j(:,:,j) = L0_A_Ji(1:3,1:3,j)*[0;0;1];
end

% Axis perpendicular to rotation of body i relative to joint j as seen in 
% link 0 frame
L0_r_JjBi = sym(zeros(3,1,n_j,n_j));
for i=1:n_j
    for j=1:n_j
        L0_r_JjBi(:,:,i,j) = cross(L0_n_j(:,:,j), L0_P_JjBi(:,:,i,j));
    end
end

% Velocity of body i relative to joint j as seen in link 0 frame
L0_dP_JjBi = sym(zeros(3,1,n_j,n_j));
for i=1:n_j
    for j=1:n_j
        L0_dP_JjBi(:,:,i,j) = L0_r_JjBi(:,:,i,j)*dtheta_i(j);
    end
end

% Velocity of body i relative to link 0 as seen in link 0 frame
L0_dP_L0Bi = sym(zeros(3,1,n_j));
for i=1:n_j
    for j=1:i
        L0_dP_L0Bi(:,:,i) = L0_dP_L0Bi(:,:,i) + L0_r_JjBi(:,:,i,j)*dtheta_i(j);
    end
end

% Jacobian of translation of body i in link 0 frame
for i=1:n_j
    L0_Jt_i(:,:,i) = jacobian(L0_dP_L0Bi(:,:,i), dtheta_i);
end
L0_Jt_0 = zeros(3,n_j);

%% Kinematics of manipulator attached to flying base
% Velocity of each body relative to the world frame as seen in the world 
% frame
for i=1:n_j
    w_dP_wBi(:,:,i) = w_dP_wL0 + ...
                      w_R_L0*L0_Jt_i(:,:,i)*dtheta_i - ...
                      2*w_R_L0*cross_prod_mat(L0_P_L0Bi(:,:,i))*G_mat*dq_sym;
end

% qR_mat = q_sym(4)*eye(4) + [-cross_prod_mat(q_sym(1:3)), q_sym(1:3); -q_sym(1:3)', 0];
qR_mat = q_sym(1)*eye(4) + [0, -q_sym(2:4)'; q_sym(2:4), -cross_prod_mat(q_sym(2:4))];
% Angular velocity of each body relative to the world frame as seen in the
% world frame
for i=1:n_j
    w_Om_wBi(:,:,i) = 2*qR_mat'*dq_sym + ...
                      [zeros(1,n_j); w_A_L0(1:3,1:3)*L0_Jom_i(:,:,i)]*dtheta_i;
    w_Om_wBi(1,:,i) = 0;
end

%% System dynamics
% Generalized coordinates
xi = [w_P_wL0; q_sym; theta_i(1:end-1)];

% Generalized velocities
dxi = [w_dP_wL0; dq_sym; dtheta_i];

% State vector
z = [xi; dxi];

% System jacobian for translation of body 0
w_Jt_0 = [eye(3), -2*w_R_L0*cross_prod_mat(zeros(3,1))*G_mat, w_R_L0*L0_Jt_0];

% System jacobian for translation of body i (starting from i=1)
for i=1:n_j
    w_Jt_i(:,:,i) = [eye(3), -2*w_R_L0*cross_prod_mat(L0_P_L0Bi(:,:,i))*G_mat, w_R_L0*L0_Jt_i(:,:,i)];
end

% System jacobian for rotation of body 0
w_Jom_0 = [zeros(4,3), 2*qR_mat', [zeros(1,n_j); w_R_L0*L0_Jom_0]];
% w_Jom_0(1,:) = zeros(1,7+n_j);

% System jacobian for rotation of body i
for i=1:n_j
    w_Jom_i(:,:,i) = [zeros(4,3), 2*qR_mat', [zeros(1,n_j); w_R_L0*L0_Jom_i(:,:,i)]];
end

% Large system jacobian
% Sorry about the unreadable indexing - just trust it!
w_J = sym(zeros(7*n_b, 7+n_j));
w_J(1:3,:) = w_Jt_0;
w_J((3*n_b)+1:(3*n_b)+4,:) = w_Jom_0;
for i=1:n_j
    w_J(i*3+1:i*3+3,:) = w_Jt_i(:,:,i);
    w_J((3*n_b)+i*4+1:(3*n_b)+i*4+4,:) = w_Jom_i(:,:,i);
end

% Mass matrices
for i=1:n_b
    M_m(:,:,i) = eye(3)*m_vec(i);
end

% Inertia tensors in world frame
w_Theta_mat(:,:,1) = [1, zeros(1,3); zeros(3,1), w_A_L0(1:3,1:3)*Phi(:,:,1)*w_A_L0(1:3,1:3)'];
% w_Theta_mat(:,:,1) = [1, zeros(1,3); zeros(3,1), Phi(:,:,1)];
w_R_Bi(:,:,1) = w_A_L0(1:3,1:3);
for i=1:n_j
    w_R_Bi(:,:,i+1) = w_A_L0(1:3,1:3)*L0_A_Bi(1:3,1:3,i);
    w_Theta_mat(:,:,i+1) = [1, zeros(1,3); zeros(3,1), w_R_Bi(:,:,i+1)*Phi(:,:,i+1)*w_R_Bi(:,:,i+1)']; % TODO: TRY WITH PARALLEL AXIS THEOREM?!
%     w_Theta_mat(:,:,i+1) = [1, zeros(1,3); zeros(3,1), Phi(:,:,i+1)]; % TODO: TRY WITH PARALLEL AXIS THEOREM?!
end
w_Theta_mat = simplify(w_Theta_mat);

% Generalized inertia matrix
M_L = zeros(7*n_b);
M_L = blkdiag(M_m(:,:,1),M_m(:,:,2),M_m(:,:,3),w_Theta_mat(:,:,1), w_Theta_mat(:,:,2), w_Theta_mat(:,:,3));

%% System mass matrix
tic
M = simplify(w_J'*M_L*w_J);
toc
num_ops_M = numel(regexp(char(M), '[+-*/^]?'))

%% Coriolis matrix
tic
C = christoffel_sym_coriolis(M, xi, dxi);
C_sim = simplify(C);
toc
num_ops_C = numel(regexp(char(C_sim), '[+-*/^]?'))

%% Gravitational terms
% Position of body i relative to inertial frame as seen from inertial frame
w_P_wBi(:,:,1) = w_P_wL0;
for i=1:n_j
    w_P_wBi(:,:,i+1) = w_P_wL0 + w_A_L0(1:3,1:3)*L0_P_L0Bi(:,:,i);
end

% Homogeneous transformation of body i relative to inertial frame as seen
% from inertial frame
for i=1:n_b
    w_A_Bi(:,:,i) = [w_R_Bi(:,:,i), w_P_wBi(:,:,i); 0 0 0 1];
end

% Potential energy
E_pot = 0;
for i=1:n_b
    E_pot = E_pot + m_vec(i)*[0,0,g]*w_P_wBi(:,:,i);
end

% Vector of gravitational terms
g_bar = jacobian(E_pot, xi)';

%% Force mapping
% Mapping between local body forces and forces acting in the generalized
% coordinates by applying the principle of virtual work

% Mapping matrix
M_F = blkdiag(w_R_L0, 2*G_mat', eye(n_j));

%% Actuation forces
% Input symbols
om_sym = sym('om', [4,1], 'real');
tau_j_sym = sym('tau', [2,1], 'real');

% Thrust of rotor i link 0 frame
for i=1:4
    L0_f_thrusti(:,:,i) = [0;0;-1]*k_t*om_sym(i)^2;
end

% Thrust force in link 0 frame with rotation speed input
L0_f_xyz = sum(L0_f_thrusti, 3);

% Torque direction for each rotor
rot_dir = [-1; 1; -1; 1]; % {CW, CCW} == {1, -1}

% Torque from drag on rotor i in link 0 frame
for i=1:4
    L0_f_dragi(:,:,i) = [0; 0; -1]*k_tau*om_sym(i)*rot_dir(i);
end

% Torque on body 0 in link 0 frame with rotation speed input
L0_f_q = sym(zeros(3,1));
for i=1:4
    L0_f_q = L0_f_q + cross(S(:,i), L0_f_thrusti(:,:,i)) + L0_f_dragi(:,:,i);
end

f_x = M_F*[L0_f_xyz; L0_f_q; tau_j_sym];

%% Constraint forces
% Generalized force performing virtual work to ensure constraints are
% satisfied

% Constraint equation
phi_bar = sum(q_sym.^2) - 1; % == 0

% Constraint Jacobian
J_phi = jacobian(phi_bar, xi);

% Corrective terms
% syms t Q0(t) Q1(t) Q2(t) Q3(t)
% Qt_sym = [Q0; Q1; Q2; Q3];
% PHI_BAR = sum(Qt_sym.^2) - 1;
% b_q = -J_phi*dxi;
% -2*diff(jacobian(PHI_BAR, Qt_sym), t)
% -2*jacobian(phi_bar, xi)
% b_v = -dxi'*jacobian(J_phi,xi)*dxi 

%% Generate functions
M_func = matlabFunction(M);
C_func = matlabFunction(C_sim);
g_bar_func = matlabFunction(g_bar);
f_x_func = matlabFunction(f_x);
J_phi_func = matlabFunction(J_phi);

% For plotting
w_A_EE = w_A_Li(:,:,end)*[eye(3), [0;l2;0]; 0 0 0 1];
w_T_Li(:,:,1) = w_A_L0;
w_T_Li(:,:,2:n_l) = w_A_Li;
w_T_Li(:,:,n_l+1) = w_A_EE;
w_T_Li_func = matlabFunction(w_T_Li);


% dyn_func = @(x,u)( (M_func(x(4), x(5), x(6), x(7), x(8), x(9))) \ ...
%     (f_x_func(u(1), u(2), u(3), u(4), x(4), x(5), x(6), x(7), u(5), u(6)) - ...
%     C_func(x(13), x(14), x(15), x(16), x(17), x(18), x(4), x(5), x(6), x(7), x(8), x(9), x(10), x(11), x(12))*x(10:end) - ...
%     g_bar_func(x(4), x(5), x(6), x(7), x(8), x(9))) );

%% Simulation setup
% Control sequence
omega_rotor = [sqrt(0.25*(m0+m1+m2)*g/k_t); 
               sqrt(0.25*(m0+m1+m2)*g/k_t); 
               sqrt(0.25*(m0+m1+m2)*g/k_t); 
               sqrt(0.25*(m0+m1+m2)*g/k_t)].*ones(4, N_t);
tau_servo = [0.1; 0].*ones(2, N_t);
u = [omega_rotor; tau_servo];

% State vector
R0 = ext_rot_mat_xyz(pi, 0, 0); % Initial attitude matrix, quadrotor rotation in inertial frame
%plot_frame_rotation(R0);

q0 = R2q(round(R0)');   % Initial attitude quaternion [-]
q0 = [0; 1; 0; 0];
dq0 = [0; 0; 0; 0];
p_B0 = [0; 0; 0.5];     % Initial position [m]
om_B0 = [0; 0; 0];      % Initial UAV angular velocity [rad/s]
v_B0 = [0; 0; 0];       % Initial UAV velocity [m/s]
th1_0 = 0;              % Initial joint 1 position [rad]
th2_0 = 0;              % Initial joint 2 position [rad]
dth1_0 = 0;             % Initial joint 1 velocity [rad/s]
dth2_0 = 0;             % Initial joint 2 velocity [rad/s]

x0 = [p_B0; q0; th1_0; th2_0; v_B0; dq0; dth1_0; dth2_0];
am_plot(x0, w_A_L0, w_A_Bi, w_A_Ji, w_A_Li, 4)

tic
dx0 = solve_constrained_el(x0, u(:,1), M_func, C_func, g_bar_func, f_x_func, J_phi_func)
% dx0 = dyn_func(x0, u(:,1))
toc

% tic
% w_J0 = w_J_func(x0(4), x0(5), x0(6), x0(7), x0(8), x0(9));
% M_L0 = M_L_func(x0(4), x0(5), x0(6), x0(7), x0(8), x0(9));
% M0a = w_J0'*M_L0*w_J0;
% toc
% tic
% M0 = M_func(x0(4), x0(5), x0(6), x0(7), x0(8), x0(9));
% toc
% M0I = inv(M0);
% f_x0 = f_x_func(u(1,1), u(2,1), u(3,1), u(4,1), x0(4), x0(5), x0(6), x0(7), u(5,1), u(6,1))
% L0_f_q0 = L0_f_q_func(u(1,1), u(2,1), u(3,1), u(4,1))
% M0\(f_x_func(u(1,1), u(2,1), u(3,1), u(4,1), x0(4), x0(5), x0(6), x0(7), u(5,1), u(6,1)).*[1; 1; 1; 0; 0; 0; 0; 1; 1])
% C_func(x0(13), x0(14), x0(15), x0(16), x0(17), x0(18), x0(4), x0(5), x0(6), x0(7), x0(8), x0(9), x0(10), x0(11), x0(12))*x0(10:end)
% -(M0\g_bar_func(x0(4), x0(5), x0(6), x0(7), x0(8), x0(9)))

% Initialize simulation variables
x_sim = [x0, zeros(length(x0), N_t-1)];
dx_sim = [dx0, zeros(length(dx0), N_t-1)];
qn = [norm(q0), zeros(1,N_t-1)];

%% Simulation
tic
for i = 2:N_t
    x_sim(:,i) = x_sim(:,i-1) + t_s*dx_sim(:,i-1);
    x_sim(4:7,i) = x_sim(4:7,i)./norm(x_sim(4:7,i)); % Renormalization
    dx_sim(:,i) = solve_constrained_el(x_sim(:,i), u(:,i), M_func, C_func, g_bar_func, f_x_func, J_phi_func);
    %dx_sim(:,i) = [x_sim(10:end,i); dyn_func(x_sim(:,i), u(:,i))];
    %dx_sim(:,i) = am_state_dot_func(tau_servo(1,i), tau_servo(2,i), x_sim(15,i), x_sim(17,i), x_sim(1,i), x_sim(2,i), x_sim(3,i), x_sim(4,i), x_sim(14,i), x_sim(16,i), omega_rotor(1,i), omega_rotor(2,i), omega_rotor(3,i), omega_rotor(4,i), x_sim(8,i), x_sim(9,i), x_sim(10,i), x_sim(11,i), x_sim(12,i), x_sim(13,i));
    % Quaternion norm
    qn(:,i) = norm(x_sim(4:7,i));
end
toc

uam_animate(x_sim(1:3,:), x_sim(4:7,:), x_sim(8:9,:), w_T_Li_func, S, t_s, 4)
% am_plot(x_sim(:,400), w_A_L0, w_A_Bi, w_A_Ji, w_A_Li, 4)
%% Plots
close all
% UAV states
figure(1);
subplot(2, 2, 1);
hold on;
plot(T_sim, x_sim(4:7,:))
xlabel("Time: $t$ [s]")
ylabel("Attitude: $\mathbf{q}_q$ [-]");
title("Attitude quaternion")
legend("$q_0$", "$q_1$", "$q_2$", "$q_3$", 'FontSize', LEGEND_FONT_SIZE)
% plot(T_sim, quat2eul(q2mlq(x_sim(1:4,:))))
% xlabel("Time: $t$ [s]")
% ylabel("Attitude: $\mathbf{\eta}$ [rad]");
% title("Attitude in Euler angles (ZYX)")
% legend("$\phi$", "$\theta$", "$\psi$", 'FontSize', LEGEND_FONT_SIZE)

subplot(2,2,2);
hold on;
plot(T_sim, x_sim(1:3,:))
xlabel("Time: $t$ [s]")
ylabel("Displacement: $\mathbf{r}_B$ [m]");
title("Position (inertial frame)")
legend("$r_x$", "$r_y$", "$r_z$", 'FontSize', LEGEND_FONT_SIZE)

% subplot(2,2,3);
% hold on;
% plot(T_sim, x_sim(8:10,:))
% xlabel("Time: $t$ [s]")
% ylabel("Angular velocity: $\mathbf{\omega}_B$ [rad/s]");
% title("Angular velocity")
% legend("$\omega_x$", "$\omega_y$", "$\omega_z$", 'FontSize', LEGEND_FONT_SIZE)
subplot(2,2,3);
hold on;
plot(T_sim, x_sim(13:16,:))
xlabel("Time: $t$ [s]")
ylabel("Attitude velocity: $\mathbf{\dot q}_q$ [rad/s]");
title("Attitude velocity")
legend("$\dot q_0$", "$\dot q_1$", "$\dot q_2$", "$\dot q_3$", 'FontSize', LEGEND_FONT_SIZE)

subplot(2,2,4);
hold on;
plot(T_sim, x_sim(10:12,:));
xlabel("Time: $t$ [s]")
ylabel("Velocity: $\mathbf{\nu}_B$ [m/s]");
title("Velocity (inertial frame)")
legend("$\nu_x$", "$\nu_y$", "$\nu_z$", 'FontSize', LEGEND_FONT_SIZE)
hold off;

% UAV state derivatives 
figure(2); 
subplot(2,2,1);
hold on;
plot(T_sim, dx_sim(4:7,:))
xlabel("Time: $t$ [s]")
ylabel("Attitude: $\mathbf{q}_q$ [-]");
title("Attitude quaternion kinematics")
legend("$\dot q_1$", "$\dot q_2$", "$\dot q_3$", "$\dot q_4$", 'FontSize', LEGEND_FONT_SIZE)

subplot(2,2,2);
hold on;
plot(T_sim, dx_sim(1:3,:))
xlabel("Time: $t$ [s]")
ylabel("Velocity: $\mathbf{\dot r}_B$ [m/s]");
title("Velocity (inertial frame)")
legend("$\dot r_x$", "$\dot r_y$", "$\dot r_z$", 'FontSize', LEGEND_FONT_SIZE)

subplot(2,2,3);
hold on;
plot(T_sim, dx_sim(13:16,:))
xlabel("Time: $t$ [s]")
ylabel("Attitude acceleration: $\mathbf{\dot \omega}_B$ [rad/s$^2$]");
title("Attitude acceleration")
legend("$\dot \omega_x$", "$\dot \omega_y$", "$\dot \omega_z$", 'FontSize', LEGEND_FONT_SIZE)

subplot(2,2,4);
hold on;
plot(T_sim, dx_sim(10:12,:));
xlabel("Time: $t$ [s]")
ylabel("Acceleration: $\mathbf{\dot \nu}_B$ [m/s$^2$]");
title("Acceleration (inertial frame)")
legend("$\dot \nu_x$", "$\dot \nu_y$", "$\dot \nu_z$", 'FontSize', LEGEND_FONT_SIZE)
hold off;

% Manipulator states and derivatives
figure(3); 
subplot(3, 1, 1);
hold on;
plot(T_sim, x_sim(8:9,:))
xlabel("Time: $t$ [s]")
ylabel("Angle: $\theta_1, \; \theta_2$ [rad]");
title("Joint position")
legend("$\theta_1$", "$\theta_2$", 'FontSize', LEGEND_FONT_SIZE)

subplot(3,1,2);
hold on;
plot(T_sim, x_sim(17:18,:))
xlabel("Time: $t$ [s]")
ylabel("Angular velocity: $\dot{\theta}_1, \; \dot{\theta}_2$ [rad/s]");
title("Joint velocity")
legend("$\dot{\theta}_1$", "$\dot{\theta}_2$ ", 'FontSize', LEGEND_FONT_SIZE)

subplot(3, 1, 3);
hold on;
plot(T_sim, dx_sim(17:18,:))
xlabel("Time: $t$ [s]")
ylabel("Angular acceleration: $\ddot{\theta}_1, \; \ddot{\theta}_2$ [rad/s$^2$]");
title("Joint acceleration")
legend("$\ddot{\theta}_1$", "$\ddot{\theta}_2$", 'FontSize', LEGEND_FONT_SIZE)

