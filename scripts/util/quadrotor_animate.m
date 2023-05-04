function quadrotor_animate(pos, att, u, S, t_s, fig_no)
    % Define visualization parameters
    %scale = 0.1; % scale factor for quadcopter size
    %fps = 30; % frames per second
    %duration = size(pos,1)/fps; % duration of animation
    %dt = 1/fps; % time step
    dt = t_s; % Actual time step
    duration = length(pos)*dt;
    
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
    color_list = ['r', 'g', 'b'];

    % TODO: Compute all quantities prior to animating
    % TODO: Add time to title as you did with satellite orbit plot
    
    % Animate quadcopter
    for i = 1:10:length(x)
        % Extract position and orientation at current time step
        x0 = x(i);
        y0 = y(i);
        z0 = z(i);
%         phi0 = phi(i); % roll
%         theta0 = theta(i); % pitch
%         psi0 = psi(i); % yaw

        % Define rotation matrices
%         Rz = [cos(psi0) -sin(psi0) 0; sin(psi0) cos(psi0) 0; 0 0 1];
%         Ry = [cos(theta0) 0 sin(theta0); 0 1 0; -sin(theta0) 0 cos(theta0)];
%         Rx = [1 0 0; 0 cos(phi0) -sin(phi0); 0 sin(phi0) cos(phi0)];
%         R=eul2rotm([psi0, theta0, phi0], "ZYX");

        % Coordinate frame at inertial origin
        render_list = [zeros(3,1); 1; 0; 0]; % W_x
        render_list = [render_list, [zeros(3,1); 0; 1; 0]]; % W_y
        render_list = [render_list, [zeros(3,1); 0; 0; 1]]; % W_z

        % Coordinate frame at CoM
        Bx = rod_qv_rotate(att(:,i), [1; 0; 0]);
        By = rod_qv_rotate(att(:,i), [0; 1; 0]);
        Bz = rod_qv_rotate(att(:,i), [0; 0; 1]);
        render_list = [render_list, [x0; y0; z0; Bx], [x0; y0; z0; By], [x0; y0; z0; Bz]];
        
        % Rotor vectors
        Bs1 = rod_qv_rotate(att(:,i), S(:,1));
        Bs2 = rod_qv_rotate(att(:,i), S(:,2));
        Bs3 = rod_qv_rotate(att(:,i), S(:,3));
        Bs4 = rod_qv_rotate(att(:,i), S(:,4));
        render_list = [render_list, [x0; y0; z0; Bs1], [x0; y0; z0; Bs2], [x0; y0; z0; Bs3], [x0; y0; z0; Bs4]];
        
        % Thrust force
        F_t = rod_qv_rotate((att(:,i)), u(:,i));
        render_list = [render_list, [x0; y0; z0; F_t]];
        
        % Torque vectors
        
        % Force of gravity
        
        % Plot quadcopter
        clf(fig_no);
        hold on;
        view(3);
        plot3(x(1:i), y(1:i), z(1:i), 'k:');
        quiver3(render_list(1,[1,4]), render_list(2,[1,4]), render_list(3,[1,4]), render_list(4,[1,4]), render_list(5,[1,4]), render_list(6,[1,4]), 'r', 'AutoScale', 'off');
        quiver3(render_list(1,[2,5]), render_list(2,[2,5]), render_list(3,[2,5]), render_list(4,[2,5]), render_list(5,[2,5]), render_list(6,[2,5]), 'g', 'AutoScale', 'off');
        quiver3(render_list(1,[3,6]), render_list(2,[3,6]), render_list(3,[3,6]), render_list(4,[3,6]), render_list(5,[3,6]), render_list(6,[3,6]), 'b', 'AutoScale', 'off');
        quiver3(render_list(1,7:10), render_list(2,7:10), render_list(3,7:10), render_list(4,7:10), render_list(5,7:10), render_list(6,7:10), 'AutoScale', 'off', 'Color', 'k', 'LineWidth', 2, 'ShowArrowHead','off');
        quiver3(render_list(1,11), render_list(2,11), render_list(3,11), render_list(4,11), render_list(5,11), render_list(6,11), 'c', 'AutoScale', 'off');
        axis equal;
        drawnow;
        % Pause to control animation speed
        %pause(dt);
    end
end