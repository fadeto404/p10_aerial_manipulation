function am_plot(x, w_A_L0, w_A_Bi, w_A_Ji, w_A_Li, fig_no)
    % Inertial frame
    render_list = [zeros(3,3); eye(3)]; 

    w_A_L0_func = matlabFunction(w_A_L0);
    w_A_Bi_func = matlabFunction(w_A_Bi);
    w_A_Ji_func = matlabFunction(w_A_Ji);
    w_A_Li_func = matlabFunction(w_A_Li);

    w_A_L0_res = w_A_L0_func(x(4), x(5), x(6), x(7), x(1), x(2), x(3));
    w_A_Bi_res = w_A_Bi_func(x(4), x(5), x(6), x(7), x(8), x(9), x(1), x(2), x(3));
    w_A_Ji_res = w_A_Ji_func(x(4), x(5), x(6), x(7), x(8), x(1), x(2), x(3));
    w_A_Li_res = w_A_Li_func(x(4), x(5), x(6), x(7), x(8), x(9), x(1), x(2), x(3));

    figure(fig_no); hold on;
    plotTransforms([w_A_L0_res(1:3,4)'; w_A_Ji_res(1:3,4,1)'; ...
        w_A_Ji_res(1:3,4,2)'; w_A_Bi_res(1:3,4,1)'; w_A_Bi_res(1:3,4,2)'; w_A_Bi_res(1:3,4,3)'], ...
        [rotm2quat(w_A_L0_res(1:3,1:3)); rotm2quat(w_A_Ji_res(1:3,1:3,1)); rotm2quat(w_A_Ji_res(1:3,1:3,2)); ...
        rotm2quat(w_A_Bi_res(1:3,1:3,1)); rotm2quat(w_A_Bi_res(1:3,1:3,2)); rotm2quat(w_A_Bi_res(1:3,1:3,3))],"FrameSize",0.05)
    axis equal;
end