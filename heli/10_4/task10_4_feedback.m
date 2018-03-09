clc
clear all
%% Initialization and model definition
 % Change this to the init file corresponding to your helicopter

init

% Discrete time system model. x = [lambda r p p_dot]'

Ac_2 = [0 1 0 0 0 0; 
    0 0 -K_2 0 0 0; 
    0 0 0 1 0 0; 
    0 0 -K_1*K_pp -K_1*K_pd 0 0;
    0 0 0 0 0 1; 
    0 0 0 0 -K_3*K_ep -K_3*K_ed];
Bc_2 = [0 0;
    0 0;
    0 0;
    K_1*K_pp 0;
    0 0; 
    0 K_3*K_ep];

delta_t	= 0.25; % sampling time
A2 = (eye(6)+delta_t*Ac_2);
B2 = delta_t*Bc_2;

% Number of states and inputs
mx = size(A2,2); % Number of states (number of columns in A)
mu = size(B2,2); % Number of inputs(number of columns in B)

% Initial values
x1_0 = pi;                              % Lambda
x2_0 = 0;                               % r
x3_0 = 0;                               % p
x4_0 = 0;                               % p_dot
x5_0 = 0;                               % e
x6_0 = 0;                               % e_dot
x0 = [x1_0 x2_0 x3_0 x4_0 x5_0 x6_0]';           % Initial values

% Time horizon and initialization
global N  
N = 40;                                  % Time horizon for states
M  = N;                                 % Time horizon for inputs
z  = zeros(N*mx+M*mu,1);                % Initialize z for the whole horizon
z0 = z;                                 % Initial value for optimization
z0(1) = x1_0;

% Bounds
lower 	    = -30*pi/180;                   % Lower bound on control
upper	    = 30*pi/180;                   % Upper bound on control

xl      = -Inf*ones(mx,1);              % Lower bound on states (no bound)
xu      = Inf*ones(mx,1);               % Upper bound on states (no bound)
ul      = -Inf*ones(mu,1);              % Lower bound on inputs (no bound)
uu      = Inf*ones(mu,1); 
xl(3)   = lower;                           % Lower bound on state x3
xu(3)   = upper;                           % Upper bound on state x3
ul(1)   = lower;                        % Lower bound on pitch
uu(1)   = upper;                        % Upper bound on pitch

% Generate constraints on measurements and inputs
[vlb,vub]       = gen_constraints(N,M,xl,xu,ul,uu); % hint: gen_constraints
vlb(N*mx+M*mu)  = 0;                    % We want the last input to be zero
vub(N*mx+M*mu)  = 0;                    % We want the last input to be zero

%% Generate system matrixes for equality constraints
 Aeq = gen_aeq(A2,B2,N,mx,mu);                % Generate A, hint: gen_aeq
 beq = zeros(1,N*mx);                          % Generate b (40 entries)
 beq(1,1:6) = A2*x0;                        % ??? 
 
%%
q1 = 1;
q2 = 1;
Q = diag([1 0 0 0 0 0]);
P = diag([q1 q2]);

H = gen_q(Q,P,N,M);
  

objfun = @(z) z'*H*z;

%% Generate system matrices for inequality constraints

options = optimoptions('fmincon');
options.Algorithm = 'sqp';

z = fmincon(objfun, z0, [], [], Aeq, beq, vlb, vub, @nonlcon, options); 

%% Extract control inputs and states
u1  = [z(N*mx+1:mu:N*mx+M*mu-1);z(N*mx+M*mu-1)]; % Control input from solution
u2  = [z(N*mx+2:mu:N*mx+M*mu);z(N*mx+M*mu)];



x1 = [x0(1);z(1:mx:N*mx)];              % State x1 from solution
x2 = [x0(2);z(2:mx:N*mx)];              % State x2 from solution
x3 = [x0(3);z(3:mx:N*mx)];              % State x3 from solution
x4 = [x0(4);z(4:mx:N*mx)];              % State x4 from solution
x5 = [x0(5);z(5:mx:N*mx)];
x6 = [x0(6);z(6:mx:N*mx)];

num_variables = 5/delta_t;
zero_padding = zeros(num_variables,1);
unit_padding  = ones(num_variables,1);

u1   = [zero_padding; u1; zero_padding];
u2  = [zero_padding; u2; zero_padding];
x1  = [pi*unit_padding; x1; zero_padding];
x2  = [zero_padding; x2; zero_padding];
x3  = [zero_padding; x3; zero_padding];
x4  = [zero_padding; x4; zero_padding];
x5  = [zero_padding; x5; zero_padding];
x6  = [zero_padding; x6; zero_padding];
x_optimal = [x1 x2 x3 x4 x5 x6];
u_optimal = [u1,u2];
t = 0:delta_t:delta_t*(length(u1)-1);

%% LQR
q_1 = 10;
q_2 = 1;
q_3 = 1;
q_4 = 1;
q_5 = 10;
q_6 = 1;

r_1 = 1;
r_2 = 0.1;

Q_lqr = blkdiag(q_1, q_2, q_3, q_4, q_5, q_6);
R_lqr = blkdiag(r_1, r_2);


[K,S,e] = dlqr(A2, B2, Q_lqr, R_lqr, []);

figure
subplot(811)
stairs(t,u1),grid
ylabel('u1')
subplot(812)
stairs(t,u2),grid
ylabel('u2')
subplot(813)
plot(t,x1,'m',t,x1,'mo'),grid
ylabel('lambda')
subplot(814)
plot(t,x2,'m',t,x2','mo'),grid
ylabel('r')
subplot(815)
plot(t,x3,'m',t,x3,'mo'),grid
ylabel('p')
subplot(816)
plot(t,x4,'m',t,x4','mo'),grid
ylabel('pdot')
subplot(817)
plot(t, x5, 'm', t, x5, 'mo'), grid
ylabel('e')
subplot(818)
plot(t, x6, 'm', t, x6, 'mo'), grid
ylabel('edot')
xlabel('tid (s)')


