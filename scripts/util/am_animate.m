function am_animate(pos, att, dh_table, Theta, u, S, t_s, fig_no)
    % Define visualization parameters
    %scale = 0.1; % scale factor for quadcopter size
    %fps = 30; % frames per second
    %duration = size(pos,1)/fps; % duration of animation
    %dt = 1/fps; % time step
    dt = t_s; % Actual time step
    duration = length(pos)*dt;
    N = size(pos, 2);
    
    x = pos(1,:);
    y = pos(2,:);
    z = pos(3,:);
    
    % Create figure window
    figure(fig_no);
    %axis([-2 2 -2 2 -2 2]);
    xlabel('x');
    ylabel('y');
    zlabel('z');

    % Create list of rendered arrow coordinates
    render_list = []; % X Y Z U V W

    % TODO: Compute all quantities prior to animating
    % TODO: Add time to title as you did with satellite orbit plot
    % Forward kinematics with DH-parameters
    [T_0t_sym, T_i_sym] = dh_forward_kinematics(dh_table);
    T_0t_func = matlabFunction(T_0t_sym);
    T_01_func = matlabFunction(T_i_sym(:,:,1));
    T_12_func = matlabFunction(T_i_sym(:,:,2));
    T_2t_func = matlabFunction(T_i_sym(:,:,3));
    
    frame_array = zeros(6, 15, N);
    link_array = zeros(3, 3, N);
    quadrotor_array = zeros(6, 4, N);
    % Compute all objects
    for i=1:N
        th1 = Theta(1,i);
        th2 = Theta(2,i);

        % Link transformations
        Tw0 = [q2R(att(:,i))', pos(:,i); [zeros(1,3), 1]];
        T01 = Tw0*T_01_func(th1);
        T02 = T01*T_12_func(th2);
        T0t = Tw0*T_0t_func(th1, th2);
        
        % Link positions
        p0 = Tw0(1:3,4);
        p1 = T01(1:3,4); 
        p2 = T0t(1:3,4);
        link_array(:,:,i) = [p0, p1, p2];

        % Rotor vectors
        Bs1 = rod_qv_rotate(att(:,i), S(:,1));
        Bs2 = rod_qv_rotate(att(:,i), S(:,2));
        Bs3 = rod_qv_rotate(att(:,i), S(:,3));
        Bs4 = rod_qv_rotate(att(:,i), S(:,4));
        quadrotor_array(:,:,i) = [[p0; Bs1], [p0; Bs2], [p0; Bs3], [p0; Bs4]];
        
        % Frame rotations
        R0 = Tw0(1:3,1:3);
        R1 = T01(1:3,1:3);
        R2 = T02(1:3,1:3);
        R3 = T0t(1:3,1:3);

        % Create list of objects to render
        render_list = [zeros(3,3); eye(3)]; % World frame
        render_list = [render_list, [ones(3,3).*p0; R0]]; % Base frame
        render_list = [render_list, [ones(3,6).*p1; R1, R2]]; % Joint 1+2 frames
        render_list = [render_list, [ones(3,3).*p2; R3]]; % Tool frame
        frame_array(:,:,i) = render_list;
    end

    % Prepare plotting
    figure(fig_no); hold on;
    xlabel('$X_w$')
    ylabel('$Y_w$')
    zlabel('$Z_w$')
    title('2-joint aerial manipulator')
    view(3);
    
    % Start animating
    tic
    anim_step = 5;
    for i=1:anim_step:size(Theta, 2)
        % Get current render list
        p0 = link_array(:,1,i);
        p1 = link_array(:,2,i);
        p2 = link_array(:,3,i);
        render_list = frame_array(:,:,i);
        
        % Prepare figure
        clf(fig_no);
        hold on;
        xlabel('$X_w$')
        ylabel('$Y_w$')
        zlabel('$Z_w$')
        title('2-joint aerial manipulator')
        view(3);
        
        % Plot quadrotor vectors
        quiver3(quadrotor_array(1,:,i), ...
                quadrotor_array(2,:,i), ...
                quadrotor_array(3,:,i), ...
                quadrotor_array(4,:,i), ...
                quadrotor_array(5,:,i), ...
                quadrotor_array(6,:,i), ...
                'AutoScale', 'off', 'Color', 'k', 'LineWidth', 2, 'ShowArrowHead','off');

        % Plot links
        plot3([p0(1), p1(1)], [p0(2), p1(2)], [p0(3), p1(3)], 'Color', [0.4 0.4 0.4], 'LineWidth', 2);
        plot3([p1(1), p2(1)], [p1(2), p2(2)], [p1(3), p2(3)], 'Color', [0.4 0.4 0.4], 'LineWidth', 2);
        
        % Plot frames        
        quiver3(render_list(1,1:3:end), render_list(2,1:3:end), render_list(3,1:3:end), render_list(4,1:3:end), render_list(5,1:3:end), render_list(6,1:3:end), 0.2, 'r', 'AutoScale', 'off');
        quiver3(render_list(1,2:3:end), render_list(2,2:3:end), render_list(3,2:3:end), render_list(4,2:3:end), render_list(5,2:3:end), render_list(6,2:3:end), 0.2, 'g', 'AutoScale', 'off');
        quiver3(render_list(1,3:3:end), render_list(2,3:3:end), render_list(3,3:3:end), render_list(4,3:3:end), render_list(5,3:3:end), render_list(6,3:3:end), 0.2, 'b', 'AutoScale', 'off');

        % Plot EE path
        no_points = round(i/anim_step,0)+1;
        scatter3(reshape(link_array(1,3,1:anim_step:i),[1,no_points]), ...
                 reshape(link_array(2,3,1:anim_step:i),[1,no_points]), ...
                 reshape(link_array(3,3,1:anim_step:i),[1,no_points]), '.');
        
        axis equal
        drawnow;
        pause(0.05);
    end
    toc
end