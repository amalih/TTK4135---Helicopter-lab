%% Optimal Control of Pitch/Travel with Feedback (LQ)

template_problem_2

Ac = [0 1 0 0; 0 0 -K_2 0; 0 0 0 1; 0 0 -K_1*K_pp -K_1*K_pd];
Bc = [0;0;0;K_1*K_pp];


% Discrete time system model. x = [lambda r p p_dot]'
delta_t	= 0.25; % sampling time
A1 = (eye(4)+delta_t*Ac);
B1 = delta_t*Bc;

Q = diag([10 1 1 0.1]);
R = 1;

[K,S,e] = dlqr(A1, B1, Q, R, []);

%%x_optimal(:,1) = (x_optimal(:,1)-3.14);
%%x_optimal = x_optimal.*(-1);