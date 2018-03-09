%% Optimal Control of Pitch/Travel with Feedback (LQ)

init
task10_2


Ac = [0 1 0 0; 0 0 -K_2 0; 0 0 0 1; 0 0 -K_1*K_pp -K_1*K_pd];
Bc = [0;0;0;K_1*K_pp];


% Discrete time system model. x = [lambda r p p_dot]'
delta_t	= 0.25; % sampling time
A1 = (eye(4)+delta_t*Ac);
B1 = delta_t*Bc;

Q = diag([10 1 1 0.1]); % riktig: 100 1 1 0.1
R = 1;

[K,S,e] = dlqr(A1, B1, Q, R, [])


