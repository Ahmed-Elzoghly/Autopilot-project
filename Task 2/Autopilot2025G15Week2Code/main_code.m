clc;
close all;
clearvars;

%% ========================== Time Vector Initialization ==========================
t0 = 0;             % Initial time (s)
tf = 25;            % Final time (s)
dt = 0.001;         % Time step (s)
t_vec = t0:dt:tf;   % Time vector

%% ========================== External Forces and Moments ==========================
[Forces, Moments]  = get_forces_and_moments(); % Forces (N) Moments (N.m)
 

%% ========================== Aircraft Mass and Inertia ==========================
g = 9.81; % Gravity (m/s^2)
m = 11;   % Mass of the aircraft (kg)

% Inertia Matrix (kg.m^2)
I_mat = [1 -2 -1; 
         -2 5 -4; 
         -1 -4 0.2];

%% ========================== Initial Conditions ==========================
% Initial state vector: [u, v, w, p, q, r, φ, θ, ψ, x, y, z]
states_vec(:,1) = [10, 2, 0, 2*pi/180, pi/180, 0, 20*pi/180, 15*pi/180, 30*pi/180, 2, 4, 7];

% Assign initial values
[u0,v0,w0,p0,q0,r0,phi0,theta0,psi0,x0,y_0,z0] = deal(10,2,0,2*pi/180,pi/180,0,20*pi/180,15*pi/180,30*pi/180,2,4,7);

%% ========================== Runge-Kutta 4th Order (RK4) ==========================
[t_vec_RK4, states_vec_RK4] = raunge_kutta_4(t_vec, states_vec(:,1), Forces, Moments, m, I_mat);

%% ========================== Simulink RK4 ==========================
simOut = sim("RBD_sim_model.slx");

%% ========================== Convert Euler Angles to Degrees ==========================
% Convert RK4 results
states_vec_RK4(7:9, :) = rad2deg(states_vec_RK4(7:9, :));

% Convert Simulink results
simOut.phi_simulink.data   = rad2deg(simOut.phi_simulink.data);
simOut.theta_simulink.data = rad2deg(simOut.theta_simulink.data);
simOut.psi_simulink.data   = rad2deg(simOut.psi_simulink.data);

%% ========================== Plot Results ==========================
plotValidation(t_vec_RK4, states_vec_RK4, simOut);

