Q = diag([10 10 10 100]);   % State weights
R = 1;                     % Control weight

[K,~,~] = lqr(A,B,Q,R);

Acl = A - B*K;
sys_cl = ss(Acl,B,C,D);
