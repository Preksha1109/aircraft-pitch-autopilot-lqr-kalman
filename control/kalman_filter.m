Vd = 0.01*eye(4);   % Process noise
Vn = 0.1*eye(4);    % Measurement noise

[L,P,E] = lqe(A, eye(4), C, Vd, Vn);
