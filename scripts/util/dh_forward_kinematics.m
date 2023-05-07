function [T_ee, T] = dh_forward_kinematics(DH_table)
% Computes forward kinematics given modified Denavit-Hartenberg parameters.
% Uses the DH-parameter convention described by John J. Craig in:
% "Introduction to robotics: Mechanics and Control", 3rd. ed. (2005)
% Inputs:
%   DH_table:   n x 4 table with parameters: "a, alpha, d, theta" (Craig)
% Outputs:
%   T_ee:       4 x 4 transformation matrix from base to end-effector
%   T:          4 x 4 x n array of transformations from link i to i+1

[m,~] = size(DH_table);
T_ee = eye(4);
T = sym(zeros(4,4,m));
for i=1:m
    a_i = DH_table(i, :).('a_{i-1}');
    al_i = DH_table(i, :).('alpha_{i-1}');
    d_i = DH_table(i, :).('d_{i}');
    th_i = DH_table(i,:).('theta_{i}');
    % As alpha should always be a constant, not symbolic, we can round
    % it to remove round off errors occuring when alpha=±90deg which is
    % the most commonly used
    if (abs(al_i) == pi/2) || (abs(al_i) == 0)
        c_al_i = round(cos(al_i), 15); 
        s_al_i = round(sin(al_i), 15);
    else
        c_al_i = cos(al_i); 
        s_al_i = sin(al_i);
    end
    T(:,:,i) = [cos(th_i), -sin(th_i), 0, a_i;
           sin(th_i)*c_al_i, cos(th_i)*c_al_i, -s_al_i, -d_i*s_al_i;
           sin(th_i)*s_al_i, cos(th_i)*s_al_i, c_al_i, d_i*c_al_i;
           0, 0, 0, 1];
    T_ee = simplify(T_ee*T(:,:,i));
end
end