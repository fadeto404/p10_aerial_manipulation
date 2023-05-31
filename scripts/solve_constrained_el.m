function dx = solve_constrained_el(x, u, M_func, C_func, g_bar_func, f_x_func, J_phi_func)
    n_q = length(x)/2;
    % Paper method
%     M = M_func(x(4), x(5), x(6), x(7), x(8), x(9));
%     R = chol(M,"upper");
%     A = J_phi_func(x(4), x(5), x(6), x(7));
%     B = A/R;
%     P = R\pinv(B);
    
    % Book method
    M = M_func(x(4), x(5), x(6), x(7), x(8), x(9));
    A = J_phi_func(x(4), x(5), x(6), x(7));
    MiAT = M\A';
    P = eye(n_q) - MiAT*((A*MiAT)\A);
    
    % 
    a = M \ ...
    (f_x_func(u(1), u(2), u(3), u(4), x(4), x(5), x(6), x(7), u(5), u(6)) - ...
    C_func(x(13), x(14), x(15), x(16), x(17), x(18), x(4), x(5), x(6), x(7), x(8), x(9), x(10), x(11), x(12))*x(10:end) - ...
    g_bar_func(x(4), x(5), x(6), x(7), x(8), x(9)));
%     ddq = a;
    ddq = P*a;
    dx = [x(n_q+1:end); ddq];
end