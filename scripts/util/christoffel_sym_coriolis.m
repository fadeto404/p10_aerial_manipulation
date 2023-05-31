function [C, kartoffel] = christoffel_sym_coriolis(M, q, q_dot)
% Inputs:
%   M: n x n inertia matrix
%   q: n x 1 vector of symbolic variables
%   q_dot: n x 1 vector of symbolic variables (q_dot(1) = q(1)_dot)
% Outputs:
%   C: n x n coriolis matrix
%   kartoffel: All Christoffel symbols
    [n, m] = size(M);

    if n == m
        kartoffel = sym('Gamma', [n n n]); % order of coordinates are [j k i]
        for i = 1:n
            for j = 1:n
                for k = 1:n
                   kartoffel(j,k,i) = 1/2 * ( diff(M(i,j), q(k)) + diff(M(i,k), q(j)) - diff(M(k,j), q(i)) );
                end
            end
        end

        % q_dot^T Gamma q_dot = C(q,q_dot) q_dot => C = q_dot^T Gamma
        for i = 1:n
            C(i, :) = transpose(q_dot)*kartoffel(:,:,i);
        end
    end
end