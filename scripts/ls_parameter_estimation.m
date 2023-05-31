clear; clc; close all;
run util/plot_settings.m
%% Data 
% Data from Tmotor bench test of motor+prop combination at 16V
throttle_data = [50; 55; 60; 65; 75; 85; 100]; % [%]
torque_data = [0.07; 0.08; 0.09; 0.11; 0.13; .15; .18]; % [Nm]
thrust_data = [0.435; .527; .608; .702; .888; 1.076; 1.293]; % [kg]
current_data= [3.5; 4.6; 5.6; 6.8; 9.5; 12.3; 16.2]; % [A]
rpm_data = 2*pi*[6015; 6620; 7113; 7563; 8545; 9442; 10464]./60; % [rad/s]

% Data from Dynamixel performance graph (N-T curve)
servotorque_data = [0.04; 0.2; 0.4; 0.6; 0.8; 1; 1.2; 1.4; 1.6];
servocurrent_data = [0.14; 0.028*8; 12*0.028; 16*0.028; 20.3*0.028; 24.7*0.028; 29.5*0.028; 34.7*0.028; 41*0.028];

%% Least squares model fitting
A_throttle = throttle_data;
A_rpm = rpm_data.^2;
k_r = A_throttle\rpm_data;
k_tau = A_rpm\torque_data;
k_t = A_rpm\thrust_data;
A_servocurrent = servocurrent_data;
k_tau_servo = A_servocurrent\servotorque_data;

% For creating a "deadband" model
A_servo_affine = [A_servocurrent, ones(length(servocurrent_data),1)];
servo_coeffs = A_servo_affine\servotorque_data;

% Plot
x_plot = 0:0.1:100; % Throttle for plotting model
i_plot = 0:0.01:1.3; % Current for plotting model

figure; hold on;
scatter(throttle_data, rpm_data)
plot(x_plot, k_r(1)*x_plot)
title('RPM model')
xlabel('Throttle [\%]')
ylabel('RPM [Hz]')
hold off;

figure; hold on;
scatter(throttle_data, torque_data)
plot(x_plot, k_tau(1)*(k_r*x_plot).^2)
title('Torque model')
xlabel('Throttle [\%]')
ylabel('Torque [Nm]')
hold off;

figure; hold on;
scatter(throttle_data, thrust_data)
plot(x_plot, k_t(1)*(k_r*x_plot).^2)
title('Thrust model')
xlabel('Throttle [\%]')
ylabel('Thrust [kg]')

figure; hold on;
scatter(servocurrent_data, servotorque_data)
plot(i_plot, k_tau_servo(1)*i_plot)
title('Servo torque model')
xlabel('Current [A]')
ylabel('Torque [Nm]')
hold off;

deadband_model_data = servo_coeffs(1)*i_plot + servo_coeffs(2);
figure; hold on;
scatter(servocurrent_data, servotorque_data)
plot(i_plot, deadband_model_data.*(deadband_model_data>0))
title('Servo torque model with deadband')
xlabel('Current [A]')
ylabel('Torque [Nm]')
hold off;