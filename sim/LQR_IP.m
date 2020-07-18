%% MATLAB script for simulation of the IP with experimentation of variable leg length
%  The mathematical model of the IP system that I am using was derived by
%  Alexander Semion. The rest of this simulation was written by me during a
%  fever dream haha.

%% TODO: might need to tune Q more for wheel ending position and time it takes to reach steady state

%% system params
mp = 0.793;       % mass of the pendulum (point mass in this model)
mw_l = 0.243;     % mass of left wheel + motor
mw_r = 0.243;     % mass of right wheel + motor
mw = mw_l + mw_r; % total mass of wheels + motors
l = 0.18;             % length of pendulum (estimation, subject to change)
r = 0.045;        % radius of wheel
g = 9.81;         % gravity accel

% init state (d = derivative)
phi = 0;
phi_d = 0;
theta = 0;
theta_d = 0;

%% State Matrices (already linearized around IP balancing in place (0))
% system matrix (dynamics)
A = [0     1                0              0;
     0     0    (-1*mp*g)/(r*(mp+4*mw))    0;
     0     0                0              1;
     0     0   (g*(mp+2*mw))/(l*(mp+4*mw)) 0];

% control/input matrix
B = [       0;
     2/(r^2*(mp+4*mw));
            0;
    -1/(l*r*(mp+4*mw))];

% output matrix (we're interested in the whole state vector as we can
% measure it all)
C = [1 0 0 0;
     0 1 0 0;
     0 0 1 0;
     0 0 0 1];

% feedforward matrix (assuming no measurement noise & that our outputs do
% not directly depend on our inputs)
D = [0;
     0;
     0;
     0];

% state vector
% x = [phi;
%      phi_d
%      theta;
%      theta_d];
states = {'phi' 'phi_d' 'theta' 'theta_d'};
inputs = {'u'}; % control input (wheel motor torque)
outputs = {'phi' 'phi_d' 'theta' 'theta_d'};
n = size(states);
n = n(2:2);

%% LQR
%  calculate optimal K gain for optimal pole placement balancing Q, R
%  penalty gains
% state penalty (state vector difference to desired) aka error penalty
Q = [70  0  0   0;   % phi penalty
     0   1  0   0;   % phi dot penalty
     0   0  10  0;   % theta penalty
     0   0  0   10]; % theta dot penalty
% (control/energy penalty)
R = 1500;

% K = state feedback gains
% S = solution to Riccati equation
% e = eigenvalues of the closed loop system
[K,S,e] = lqr(A,B,Q,R);

%% Model Simulation
% create our state-space model
sys_ss = ss(A,B,C,D,'statename',states,'inputname',inputs,'outputname',outputs);

% check if our system is controllable
% if the rank of the controllability matrix ctrb(A,B) = n (the num of cols of the state vector (linearly independent))
controlalability = rank(ctrb(A,B));
if controlalability == n
    disp("Your system is controllable!");
end

% Now let's recreate our state-space model with new pole placements from
% the K gain from the lqr controller
% new dynamical matrix (new A matrix w/ feedback)
Ac = [(A-B*K)];
Bc = [B];
Cc = [C];
Dc = [D];

sys_ctrl_lqr = ss(Ac,Bc,Cc,Dc,'statename',states,'inputname',inputs,'outputname',outputs);

% time frame
t = 0:0.01:2.5; % to 5 seconds
% input size
u = 0.2 * ones(size(t));

% simulate the state-space system
% y = system response sampled at same times as the input (t)
% t = time vector used for simulation
% x = state trajectories (for state-space models)
[y,t,x] = lsim(sys_ctrl_lqr,u,t);

% plot the system results
[AX,H1,H2] = plotyy(t,y(:,1),t,y(:,2),'plot');
set(get(AX(1),'Ylabel'),'String','wheel position (m)')
set(get(AX(2),'Ylabel'),'String','pendulum angle (radians)')
set(get(AX(1),'Xlabel'),'String','time (s)')
title('Step Response with LQR Control for IP')

