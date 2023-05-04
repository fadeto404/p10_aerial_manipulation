clear; clc; close all;
%% Parameters
% Setup
run util/plot_settings.m

n_l = 2; % No. of links

% Define joint angles and link lengths
syms theta1 theta2 l0 l1 l2
L0 = 0.1; % Distance from base to joint 1 [m]
L1 = 0.1; % Distance from joint 1 to joint 2 [m]
L2 = 0.15; % Distance from joint 2 to tool [m]

% Inertia of links
J1 = diag(sym('J1_', [3,1]));
J2 = diag(sym('J2_', [3,1]));
J1 = eye(3);
J2 = eye(3);
bigJ(:,:,1) = J1;
bigJ(:,:,2) = J2;

% Link masses
m_vec = sym('m_', [1,2]);

% Denavit-Hartenberg parameters
i = [1; 2; 3]; % Frame indices (probably unnecessary)
a_i = [0; 0; 0]; % Distance from Z_i to Z_(i+1) along X_i [m]
alpha_i = [0; -pi/2; pi/2]; % Angle from Z_i to Z_(i+1) around X_i [rad]
d_i = [L0+L1; 0; L2]; % Distance from X_(i-1) to X_i along Z_i [m]
theta_i = [theta1; theta2; 0]; % Angle from X_(i-1) to X_i around Z_i [rad]
dh_table = table(i, a_i, alpha_i, d_i, theta_i, 'VariableNames', {'i', 'a_{i-1}', 'alpha_{i-1}', 'd_{i}', 'theta_{i}'});

%% Derive forward kinematics
% World to base frame transform
% T_w0 = [ext_rot_mat_xyz(pi, 0, 0), [0;0.1;0.5]; 0,0,0,1];
T_w0 = eye(4);

% Forward kinematics with DH-parameters
[T_0t_sym, T_i_sym] = dh_forward_kinematics(dh_table);
T_0t = matlabFunction(T_0t_sym);
T_01 = matlabFunction(T_i_sym(:,:,1));
T_12 = matlabFunction(T_i_sym(:,:,2));
T_2t = matlabFunction(T_i_sym(:,:,3));

% Frame rotations
R0 = eye(3);
R1_sym = T_i_sym(1:3,1:3,1);
R2_sym = R1_sym*T_i_sym(1:3,1:3,2);
R3_sym = T_0t_sym(1:3,1:3);
bigR(:,:,1) = R1_sym;
bigR(:,:,2) = R2_sym;
bigR(:,:,3) = R3_sym;

%% Inverse kinematics
syms x_t_sym y_t_sym z_t_sym
p_t_sym = T_0t_sym(1:3,4);% .*[1;-1;-1]
ik_eq = [x_t_sym; y_t_sym; z_t_sym] == p_t_sym;

% 1st approach (x and z free, y constrained)
sols1 = sym([]);
theta2eq = solve(ik_eq(3), theta2);
for i=1:2
    theta1eq_xz = subs(ik_eq(1), theta2, theta2eq(i));
    theta1eq_xz = solve(theta1eq_xz, theta1);

    sols1(2*i-1:2*i,1) = theta1eq_xz;
    sols1(2*i-1:2*i,2) = theta2eq(i);
end
ik_xz_func = matlabFunction(sols1);
% z is limited: L0+L1 <= z <= L0+L1+L2
% x is limited: -0.1*sqrt(2) <= x <= 0.1*sqrt(2)


% 2nd approach (y and z free, x constrained)
% theta1eq_y = subs(ik_eq(2), theta2, theta2eq(1));
% theta1eq_y = solve(theta1eq_y, theta1)
v = sym('v', [3,1]);
null(cross_prod_mat(v))

% 3rd approach (x and y free, z constrained)
sols3 = sym([]);
theta1eq_xy = solve(ik_eq(1), theta1);
for i=1:2
    theta2eq_xy = subs(ik_eq(2), theta1, theta1eq_xy(i));
    sols3(2*i-1:2*i,2) = solve(theta2eq_xy, theta2);
    theta1eq_xy_2 = subs(theta1eq_xy(i), theta2, sols3(2*i-1:2*i,2));
    sols3(2*i-1:2*i,1) = simplify(theta1eq_xy_2);
end
ik_xy_func = matlabFunction(sols3);

%% Dynamics 
om = sym('w',[3,n_l]);
al = sym('al', [3,n_l]);
v = sym('v', [3,n_l]);
a = sym('a', [3,n_l]);
f = sym('f', [3,n_l]);
tau = sym('tau', [3,n_l]);
joint_torques = sym('Tau', [2,1]);
dth_sym = sym('dtheta', [n_l,1]);
ddth_sym = sym('ddtheta', [n_l,1]);
P_ci = [[0; 0; -0.01], [0; -0.1; 0]]; % CoM of link i in link i frame
P_i = [[0; 0; L0+L1], [0; 0; 0], [0; -L2; 0]] ; % ^iP_i+1 Location of link i+1 frame as seen in frame i

% TODO: Extract P_i from forward kinematics/ DH-parameters
% TODO: Add rotation matrices from forward kinematics

om_0 = sym('wq_', [3,1]);
dom_0 = sym('alq_', [3,1]);
v_0 = sym('vq_', [3,1]);
dv_0 = sym('aq_', [3,1]);
f_np1 = sym('fp_', [3,1]);

ini=sym(zeros(3,1));
g_vec = -sym([0;0;-9.816]);

[tau_eq, f, n_hat] = recursive_newton_euler(theta_i(1:2), dth_sym, ddth_sym, ...
                                         m_vec, P_ci, P_i, bigJ, bigR, ini,ini,ini,g_vec)

% Counting operations (just for fun)
num_terms = numel(regexp(char(tau), '[+-*/]?'))
simplify(subs(tau, ddth_sym(2), 0))

forward_dyns = solve(simplify(tau_eq) == joint_torques, [ddth_sym(1), ddth_sym(2)])
dstate = matlabFunction([forward_dyns.ddtheta1; forward_dyns.ddtheta2])


% f = [f, f_np1];

%% Dynamics 3
% om = sym('w',[3,n_l]);
% al = sym('al', [3,n_l]);
% v = sym('v', [3,n_l]);
% a = sym('a', [3,n_l]);
% f = sym('f', [3,n_l]);
% tau = sym('tau', [3,n_l]);
% dth_sym = sym('dtheta', [n_l,1]);
% ddth_sym = sym('ddtheta', [n_l,1]);
% 
% % CoM of link i in world frame
% P_ci = [[0;0;0], ...
%         T_i_sym(1:3,1:3,1)*[0; 0; -0.01], ...
%         T_i_sym(1:3,1:3,1)*T_i_sym(1:3,1:3,2)*[0; -0.1; 0]];
% 
% % Joint axes in the world frame
% P_ji = [T_i_sym(1:3,1:3,1)*[0; 0; 1], T_i_sym(1:3,1:3,1)*T_i_sym(1:3,1:3,2)*[0; 0; 1]];
% 
% % TODO: Add rotation matrices from forward kinematics
% 
% om_0 = sym('wq_', [3,1]);
% al_0 = sym('alq_', [3,1]);
% v_0 = sym('vq_', [3,1]);
% a_0 = sym('aq_', [3,1]);
% z_0 = sym('zq_', [3,1]);
% f_np1 = sym('fp_', [3,1]);
% om = [om_0, om];
% al = [al_0, al];
% v = [v_0, v];
% a = [a_0, a];
% f = [f, f_np1];
% tau = [tau, [0;0;0]];
% 
% [joint_torques, f_l, n_hat] = recursive_newton_euler_world(theta_i(1:2), dth_sym, ddth_sym, m_vec, P_ci, bigJ, P_ji);
% exjt = expand(joint_torques);
% 
% % Get the coriolis, gravity, etc. terms
% % CG_eq = simplify(subs(joint_torques, ddth_sym(2), 0))
% % Counting operations (just for fun)
% num_terms = numel(regexp(char(joint_torques(2,1)), '[+-*/]?'))+1

%% Simulate
% Joint angles
th2 = pi/3:0.01:pi;
th1 = -pi/4 * ones(1, length(th2));

th1_0 = -pi/4;
th2_0 = pi/3;
dth1_0 = 0;
dth2_0 = 0;
u = [0.1; -0.5];

x_sim(:,1) = [th1_0; th2_0; dth1_0; dth2_0];
dx_sim(:,1) = [x_sim(3:4,1); dstate(u(1),u(2), x_sim(3,1), x_sim(4,1), 1, x_sim(2,1))];


dt = 0.001;
for i=2:20000
    x_sim(:,i) = x_sim(:,i-1) + dt*dx_sim(:,i-1);
    dx_sim(:,i) = [x_sim(3:4,i); dstate(u(1),u(2), x_sim(3,i), x_sim(4,i),1,x_sim(2,i))];
end

% 
% pd = [0.03422; 0.15];
% pd_xy = [0.03422; -0.1];
% 
% ik_check1 = ([-0.1*sqrt(2); L0+L1] <= pd) 
% ik_check2 = (pd <= [0.1*sqrt(2); L0+L1+L2])
% ik_sol = ik_xz_func(pd(1), pd(2))
% ik_sol2 = ik_xy_func(pd_xy(1), pd_xy(2))
% th1 = ik_sol2(1,1);
% th2 = ik_sol2(1,2);

Theta_sim = [x_sim(1,:); x_sim(2,:)];

manipulator_animate(dh_table, Theta_sim, 1)

%% Plotting of results
T_sim = dt*(1:length(x_sim));
figure(2); % Joint states and derivatives
subplot(3, 1, 1);
hold on;
plot(T_sim, x_sim(1:2,:))
xlabel("Time: $t$ [s]")
ylabel("Angle: $\theta_1, \; \theta_2$ [rad]");
title("Joint position")
legend("$\theta_1$", "$\theta_2$", 'FontSize', LEGEND_FONT_SIZE)

subplot(3,1,2);
hold on;
plot(T_sim, x_sim(3:4,:))
xlabel("Time: $t$ [s]")
ylabel("Angular velocity: $\dot{\theta}_1, \; \dot{\theta}_2$ [rad/s]");
title("Joint velocity")
legend("$\dot{\theta}_1$", "$\dot{\theta}_2$ ", 'FontSize', LEGEND_FONT_SIZE)

subplot(3, 1, 3);
hold on;
plot(T_sim, dx_sim(3:4,:))
xlabel("Time: $t$ [s]")
ylabel("Angular acceleration: $\ddot{\theta}_1, \; \ddot{\theta}_2$ [rad/s$^2$]");
title("Joint acceleration")
legend("$\ddot{\theta}_1$", "$\ddot{\theta}_2$", 'FontSize', LEGEND_FONT_SIZE)

%% Plot workspace
figure; hold on;
N = 60;
[x,y,z] = sphere(N);
T2 = T_2t();
r = norm(T2(1:3,4));
surf(r*x(N/2+1:end,:),r*y(N/2+1:end,:),r*z(N/2+1:end,:), 'FaceAlpha', 0.3)
title("Reachable workspace");
xlabel("$X_2$")
ylabel("$Y_2$")
zlabel("$Z_2$")
axis equal
view(3);
