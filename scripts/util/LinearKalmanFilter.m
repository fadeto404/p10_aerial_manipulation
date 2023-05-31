classdef LinearKalmanFilter < handle
    properties
        A     % State transition matrix
        B     % Control input matrix
        H     % Measurement matrix
        Q     % Process noise covariance
        R     % Measurement noise covariance
        x     % State vector
        P     % Covariance matrix
    end
    
    methods
        function obj = LinearKalmanFilter(A, B, H, Q, R)
            % Constructor
            % Inputs:
            %   A: State transition matrix
            %   B: Control input matrix
            %   H: Measurement matrix
            %   Q: Process noise covariance
            %   R: Measurement noise covariance
            
            obj.A = A;
            obj.B = B;
            obj.H = H;
            obj.Q = Q;
            obj.R = R;
            
            obj.x = [];
            obj.P = [];
        end
        
        function initState(obj, x0, P0)
            % Initialize state vector and covariance matrix
            % Inputs:
            %   x0: Initial state vector
            %   P0: Initial covariance matrix
            
            obj.x = x0;
            obj.P = P0;
        end
        
        function predict(obj, u)
            % Perform prediction step of Kalman filter
            % Inputs:
            %   u: Control input
            
            obj.x = obj.A * obj.x + obj.B * u;
            obj.P = obj.A * obj.P * obj.A' + obj.Q;
        end
        
        function correct(obj, z)
            % Perform correction step of Kalman filter
            % Inputs:
            %   z: Measurement vector
            
            S = obj.H * obj.P * obj.H' + obj.R;
            K = obj.P * obj.H' / S;
            obj.x = obj.x + K * (z - obj.H * obj.x);
            obj.P = (eye(size(obj.x, 1)) - K * obj.H) * obj.P;
        end
    end
end