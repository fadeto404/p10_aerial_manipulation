clear; clc; close all;
%% Config
run util/plot_settings.m
N_t = 1000; % No. of time samples
t_s = 0.005; % Time step size [s]
T_sim = 0:t_s:t_s*(N_t-1); % Time vector for simulation

% Parameters
m0 = 1; % Base mass [kg]
m1 = 0.1; % Link 1 mass [kg]
m2 = 0.1; % Link 2 mass [kg]
mp = 0.2; % Payload mass [kg]
I0 = eye(3); %[1, -0.1, -0.1; -0.1, 1, -0.1; -0.1, -0.1, 1]; % Base inertia [?]
I1 = eye(3); % Link 1 inertia [?]
I2 = eye(3); % Link 2 inertia [?]
Ip = eye(3); % Payload inertia [?]
k_t = 1; % Thrust constant motor [kg/%]
k_tau = 1; % Rotor drag coefficient [???]
g = 9.81; % Gravitational acceleration at Earth [m/s^2]
F_g = [0; 0; -m0*g]; % Force of gravity in inertial frame [N]

% Vectors from CoM to each motor in body frame [m]
s1 = [ 0.2;  0.2;  -0.1];
s2 = [-0.2;  0.2;  -0.1];
s3 = [-0.2; -0.2;  -0.1];
s4 = [ 0.2; -0.2;  -0.1];
S = [s1, s2, s3, s4];
% S_sym = sym('s', [3, 4]);
% S = S_sym

%% State equation
% Kinematics and dynamics
syms x_B y_B z_B x_B_dot y_B_dot z_B_dot x_B_ddot y_B_ddot z_B_ddot w_x w_y w_z dw_x dw_y dw_z
q_sym = sym('q', [4, 1]); % Attitude quaternion
q_sym_dot = sym('q_dot', [4, 1]); % Attitude quaternion derivative
Omega_sym = [w_x; w_y; w_z]; % Body angular velocity [rad/s]
omega_sym = sym('w', [4, 1]); % Rotor angular velocity [rad/s]
tau_rotor_sym = k_tau*omega_sym.^2; % Rotor torque [Nm]
f_thrust_sym = -[zeros(2,4); transpose(k_t*omega_sym.^2)]; % Thrust forces at each rotor [N]
F_total_sym = sum(f_thrust_sym, 2); % Total thrust force in body frame [N]
tau_body_sym = cross(S(:,1), f_thrust_sym(:,1)) + [0;0;tau_rotor_sym(1)]*((-1)^1); % Torque incurred in body by thrust forces and rotor torques [Nm]
for i = 2:4
    tau_body_sym = tau_body_sym + cross(S(:,i), f_thrust_sym(:,i)) + [0;0;tau_rotor_sym(i)*((-1)^i)];
end

Gamma_sym = tau_body_sym; % Quadrotor torques [Nm]
Gamma_man_sym = sym('gamma_man', [3,1]); % Manipulator torques on quadrotor [Nm]

x_sym = [q_sym; x_B; y_B; z_B; Omega_sym; x_B_dot; y_B_dot; z_B_dot];
x_sym_dot = [0.5*quat_prod_ham(x_sym(1:4), x_sym(8:10));
             rod_qv_rotate(quat_conj(x_sym(1:4)), x_sym(11:13));
             I0\(-cross(x_sym(8:10), I0*x_sym(8:10)) + Gamma_sym);
             1/m0 * (F_total_sym + rod_qv_rotate(x_sym(1:4), F_g) - cross(x_sym(8:10), m0*x_sym(8:10)))
             ];
x_sym_dot = [0.5*quat_prod_ham(x_sym(1:4), x_sym(8:10));
             x_sym(11:13);
             I0\(-cross(x_sym(8:10), I0*x_sym(8:10)) + Gamma_sym);
             1/m0 * (rod_qv_rotate(x_sym(1:4), F_total_sym) + F_g)
             ];
x_dot_func = matlabFunction(x_sym_dot);
acc_sym = x_sym_dot(11:13);
acc_func = matlabFunction(acc_sym);

% TODO: Make discrete state propagation with discrete quaternion propagation?

%% Simulation setup
% Control sequence
omega_rotor = [sqrt(0.25*m0*g/k_t); sqrt(0.26*m0*g/k_t); sqrt(0.26*m0*g/k_t); sqrt(0.25*m0*g/k_t)].*ones(4, N_t);
F_thrust = -[zeros(2, 1000); sum(omega_rotor.^2, 1)]; % For plotting only

% State vector
R0 = ext_rot_mat_xyz(pi, 0, 0); % Initial attitude matrix, quadrotor rotation in inertial frame
q0 = R2q(R0); % Initial attitude quaternion
%plot_frame_rotation(R0);
rB0 = [0; 0; 0.3]; % Initial position
x0 = [q0; rB0; zeros(6,1)]; % Initial state [q1 q2 q3 q4 x_q y_q z_q omega_x omega_y omega_z dx_q dy_q dz_q]
dx0 = x_dot_func(x0(1), x0(2), x0(3), x0(4), omega_rotor(1,1), omega_rotor(2,1), omega_rotor(3,1), omega_rotor(4,1), x0(8), x0(9), x0(10), x0(11), x0(12), x0(13));

% Initialize simulation variables
x_sim = [x0, zeros(length(x0), N_t-1)];
dx_sim = [dx0, zeros(length(x0), N_t-1)];

% Simulation
for i = 2:N_t
    x_sim(:,i) = x_sim(:,i-1) + t_s*dx_sim(:,i-1);
    x_sim(1:4,i) = x_sim(1:4,i)./norm(x_sim(1:4,i)); % Renormalization
    dx_sim(:,i) = x_dot_func(x_sim(1,i), x_sim(2,i), x_sim(3,i), x_sim(4,i), omega_rotor(1,i), omega_rotor(2,i), omega_rotor(3,i), omega_rotor(4,i), x_sim(8,i), x_sim(9,i), x_sim(10,i), x_sim(11,i), x_sim(12,i), x_sim(13,i));
end

%% Plotting of results
figure(1); % States
subplot(2, 2, 1);
hold on;
plot(T_sim, x_sim(1:4,:))
xlabel("Time: $t$ [s]")
ylabel("Attitude: $\mathbf{q}_q$ [-]");
title("Attitude quaternion")
legend("$q_1$", "$q_2$", "$q_3$", "$q_4$", 'FontSize', LEGEND_FONT_SIZE)

subplot(2,2,2);
hold on;
plot(T_sim, x_sim(5:7,:))
xlabel("Time: $t$ [s]")
ylabel("Displacement: $\mathbf{r}_B$ [m]");
title("Position")
legend("$r_x$", "$r_y$", "$r_z$", 'FontSize', LEGEND_FONT_SIZE)

subplot(2,2,3);
hold on;
plot(T_sim, x_sim(8:10,:))
xlabel("Time: $t$ [s]")
ylabel("Angular velocity: $\mathbf{\omega}_B$ [rad/s]");
title("Angular velocity")
legend("$\omega_x$", "$\omega_y$", "$\omega_z$", 'FontSize', LEGEND_FONT_SIZE)

subplot(2,2,4);
hold on;
plot(T_sim, x_sim(11:13,:));
xlabel("Time: $t$ [s]")
ylabel("Velocity: $\mathbf{\nu}_B$ [m/s]");
title("Translational velocity")
legend("$\nu_x$", "$\nu_y$", "$\nu_z$", 'FontSize', LEGEND_FONT_SIZE)
hold off;

figure(2); % State derivatives 
subplot(2, 2, 1);
hold on;
plot(T_sim, dx_sim(1:4,:))
xlabel("Time: $t$ [s]")
ylabel("Attitude: $\mathbf{q}_q$ [-]");
title("Attitude quaternion kinematics")
legend("$\dot q_1$", "$\dot q_2$", "$\dot q_3$", "$\dot q_4$", 'FontSize', LEGEND_FONT_SIZE)

subplot(2,2,2);
hold on;
plot(T_sim, dx_sim(5:7,:))
xlabel("Time: $t$ [s]")
ylabel("Velocity: $\mathbf{\dot r}_B$ [m]");
title("Velocity (inertial frame)")
legend("$\dot r_x$", "$\dot r_y$", "$\dot r_z$", 'FontSize', LEGEND_FONT_SIZE)

subplot(2,2,3);
hold on;
plot(T_sim, dx_sim(8:10,:))
xlabel("Time: $t$ [s]")
ylabel("Angular acceleration: $\mathbf{\dot \omega}_B$ [rad/s]");
title("Angular acceleration")
legend("$\dot \omega_x$", "$\dot \omega_y$", "$\dot \omega_z$", 'FontSize', LEGEND_FONT_SIZE)

subplot(2,2,4);
hold on;
plot(T_sim, dx_sim(11:13,:));
xlabel("Time: $t$ [s]")
ylabel("Acceleration: $\mathbf{\nu}_B$ [m/s]");
title("Acceleration (body frame)")
legend("$\dot \nu_x$", "$\dot \nu_y$", "$\dot \nu_z$", 'FontSize', LEGEND_FONT_SIZE)
hold off;

%figure(3); % Animation of quadrotor
pause(1);
% F_thrust = dx_sim(11:13,:);
F_thrust = -[zeros(2, 1000); sum(omega_rotor.^2, 1)];
fg = zeros(3, N_t);
for i = 1:N_t
    fg(:,i) = rod_qv_rotate(x_sim(1:4,i), F_g);
    norm(fg(:,i))
end

quadrotor_animate(x_sim(5:7,:), x_sim(1:4,:), F_thrust, S, t_s, 3)
