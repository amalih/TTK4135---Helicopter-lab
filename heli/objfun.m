function f = objfun(z)
    q1 = 1;
    q2 = 1;
    N = 40;
    M = 40;
    Q = diag([1 0 0 0 0 0]);
    P = diag([q1 q2]);
    H = gen_q(Q,P,N,M);
    
    
    f = z'*H*z;
end