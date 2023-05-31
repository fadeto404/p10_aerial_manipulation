function manipulator3_animate(dh_table, Theta, fig_no)
    % Find number of links (and joints)
    n_l = size(dh_table,1)-1;

    % Forward kinematics with DH-parameters
    [T_0e_sym, T_i_sym] = dh_forward_kinematics(dh_table);
    T_0e_func = matlabFunction(T_0e_sym);
    T_01_func = matlabFunction(T_i_sym(:,:,1));
    T_12_func = matlabFunction(T_i_sym(:,:,2));
    T_23_func = matlabFunction(T_i_sym(:,:,3));
    T_34_func = matlabFunction(T_i_sym(:,:,4));
    
    frame_array = zeros(6, (n_l+3)*3, size(Theta,2));
    link_array = zeros(3, n_l+1, size(Theta,2));
    % Compute all objects
    for i=1:size(Theta,2)
        th1 = Theta(1,i);
        th2 = Theta(2,i);
        th3 = Theta(3,i);

        % Link transformations
        T01 = T_01_func(th1);
        T02 = T01*T_12_func(th2);
        T03 = T02*T_23_func(th3);
        T0e = T_0e_func(th1, th2);
        
        % Link positions
        p0 = zeros(3,1);
        p1 = T01(1:3,4); 
        p2 = T03(1:3,4);
        p3 = T0e(1:3,4);
        link_array(:,:,i) = [p0, p1, p2, p3];
        
        % Frame rotations
        R0 = eye(3);
        R1 = T01(1:3,1:3);
        R2 = T02(1:3,1:3);
        R3 = T03(1:3,1:3);
        R4 = T0e(1:3,1:3);

        % Create list of objects to render
        render_list = [zeros(3,3); eye(3)]; % World frame
        render_list = [render_list, [ones(3,3).*p0; R0]]; % Base frame
        render_list = [render_list, [ones(3,6).*p1; R1, R2]]; % Joint 1+2 frames
        render_list = [render_list, [ones(3,3).*p2; R3]]; % Tool frame
        render_list = [render_list, [ones(3,3).*p3; R4]]; % Payload frame
        frame_array(:,:,i) = render_list;
    end


    % Prepare plotting
    figure(fig_no); hold on;
    xlabel('$X_w$')
    ylabel('$Y_w$')
    zlabel('$Z_w$')
    title('$n_l$-link manipulator in 3D')
    view(3);
    
    % Start animating
    anim_step = 50;
    for i=1:anim_step:size(Theta, 2)
        % Get current render list
        p0 = link_array(:,1,i);
        p1 = link_array(:,2,i);
        p2 = link_array(:,3,i);
        p3 = link_array(:,4,i);
        render_list = frame_array(:,:,i);
        % Plot links
        clf(fig_no);
        hold on;
        xlabel('$X_w$')
        ylabel('$Y_w$')
        zlabel('$Z_w$')
        title('$n_l$-link manipulator in 3D')
        view(3);
        plot3([p0(1), p1(1)], [p0(2), p1(2)], [p0(3), p1(3)], 'k', 'LineWidth', 2);
        plot3([p1(1), p2(1)], [p1(2), p2(2)], [p1(3), p2(3)], 'k', 'LineWidth', 2);
        plot3([p2(1), p3(1)], [p2(2), p3(2)], [p2(3), p3(3)], 'k', 'LineWidth', 2);
        
        % Plot frames        
        quiver3(render_list(1,1:3:end), render_list(2,1:3:end), render_list(3,1:3:end), render_list(4,1:3:end), render_list(5,1:3:end), render_list(6,1:3:end), 0.2, 'r', 'AutoScale', 'off');
        quiver3(render_list(1,2:3:end), render_list(2,2:3:end), render_list(3,2:3:end), render_list(4,2:3:end), render_list(5,2:3:end), render_list(6,2:3:end), 0.2, 'g', 'AutoScale', 'off');
        quiver3(render_list(1,3:3:end), render_list(2,3:3:end), render_list(3,3:3:end), render_list(4,3:3:end), render_list(5,3:3:end), render_list(6,3:3:end), 0.2, 'b', 'AutoScale', 'off');

        % Plot EE path
        no_points = round(i/anim_step,0)+1;
        scatter3(reshape(link_array(1,n_l+1,1:anim_step:i),[1,no_points]), ...
                 reshape(link_array(2,n_l+1,1:anim_step:i),[1,no_points]), ...
                 reshape(link_array(3,n_l+1,1:anim_step:i),[1,no_points]), '.');
        
        axis equal
        drawnow;
%         pause(0.01);
    end
end