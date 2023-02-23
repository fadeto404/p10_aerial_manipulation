clear; clc; close all;
% Example using the plot_frame_rotation()
run scripts/util/plot_settings.m % Requires "Current folder" to be the "p10_aerial_manipulation/" folder

theta = deg2rad(30);
R = ext_rot_mat_xyz(0, 0, theta);
plot_frame_rotation(R)