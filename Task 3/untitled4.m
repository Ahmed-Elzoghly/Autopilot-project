clc; clearvars; close all;

%% Select Plane and Flight Condition
addpath('Planes_Data');
addpath('ControlDesign');
Plane_data_file = 'B747_FCS';
%Plane_data_file = 'Jetstar_FCS';

% Read from the vector(aircraft_data) but take care of the order the values in excel sheet is arr
aircraft_data = xlsread(Plane_data_file, 'B2:B61'); % Read the excel sheet from B2 to B61

%% Time vector
dt = aircraft_data(1);
tfinal = aircraft_data(2); % Assuming tfinal is stored in the second entry of aircraft_data
time_vec = (0:dt:tfinal);
n = length(time_vec) - 1;

%% Control action
delta_aileron = 0;
delta_rudder = 0;
delta_elevator = 0;
delta_thrust = 0;

delta_control = [deg2rad([delta_aileron, delta_rudder, delta_elevator]), delta_thrust];

%% Gravity, Mass & Inertia
mass = aircraft_data(51);
gravity = aircraft_data(52);

Ixx = aircraft_data(53);
Iyy = aircraft_data(54);
Izz = aircraft_data(55);
Ixz = aircraft_data(56);
Ixy = 0;
Iyz = 0;
Inertia = [Ixx, -Ixy, -Ixz;
           -Ixy, Iyy, -Iyz;
           -Ixz, -Iyz, Izz];

%% Initial conditions
% states: [u; v; w; p; q; r; phi; theta; psi; x; y; z]
states_0 = aircraft_data(4:15);
Vtotal_0 = sqrt(states_0(1)^2 + states_0(2)^2 + states_0(3)^2); % Vtotal initial

% Forces: [Fx; Fy; Fz]
% Moments: [L; M; N]

% Initial gravitational forces
F_gravity_0 = mass * gravity * [sin(states_0(8)); ...
                               -cos(states_0(8)) * sin(states_0(7)); ...
                               -cos(states_0(8)) * cos(states_0(7))];

%% Initial moments
M_0 = [0; 0; 0];

%% Initialization
Forces = zeros(3, n+1);
Moments = zeros(3, n+1);

%% ========================== Runge-Kutta 4th Order (RK4) ==========================
[t_vec_RK4, states_vec_RK4] = raunge_kutta_4(time_vec, states_0, Forces, Moments, mass, Inertia);

%%% Delta States, forces and moments
%delta_States = states_vec_RK4 - states_0;

%% Stability derivatives for longitudinal motion
SD_long = aircraft_data(21:36);
SD_long_final = SD_long;

tempvarLong = num2cell(SD_long_final);
[Xu, Zu, Mu, Xw, Zw, Mw, Zwd, Zq, Mwd, Mq, Xde, Zde, Mde, Xdth, Zdth, Mdth] = deal(tempvarLong{:});
clear tempvarLong;

%% Stability derivatives for lateral motion
SD_lat_dash = aircraft_data(37:50);
SD_lateral_final = Lateral_correction(SD_lat_dash, Vtotal_0, Ixx, Izz, Ixz);

tempvarLat = num2cell(SD_lateral_final);
[Yv, Yb, Lb, Nb, Lp, Np, Lr, Nr, Yda, Ydr, Lda, Nda, Ldr, Ndr] = deal(tempvarLat{:});
clear tempvarLat;

% Adjust lateral stability derivatives
%Yv = Yb / Vtotal_0;
Lv = Lb / Vtotal_0;
Nv = Nb / Vtotal_0;
Yp = 0; % Not found in MACA Report
Yr = 0; % Not found in MACA Report

%% Matrices
% delta_states : [d_df d_v d_w d_p d_q d_r d_wdot]
Matrix_states = [Xu, 0, Xw, 0, 0, 0, 0;
                 0, Yv, 0, Yp, 0, Yr, 0;
                 Zu, 0, Zw, 0, Zq, 0, Zwd;
                 0, Lv, 0, Lp, 0, Lr, 0;
                 Mu, 0, Mw, 0, Mq, 0, Mwd;
                 0, Nv, 0, Np, 0, Nr, 0];

% delta_controls : [d_aileron d_elevator d_thrust d_rudder]
Matrix_controls = [0, Xde, Xdth, 0;
                   Yda, 0, 0, Ydr;
                   0, Zde, Zdth, 0;
                   Lda, 0, 0, Ldr;
                   0, Mde, Mdth, 0;
                   Nda, 0, 0, Ndr];

%% Prepare some values for state space
u0 = states_0(1);
v0 = states_0(2);
w0 = states_0(3);

p0 = states_0(4);
q0 = states_0(5);
r0 = states_0(6);

phi0 = states_0(7);
theta0 = states_0(8);
psi0 = states_0(9);

x0 = states_0(10);
y0 = states_0(11);
z0 = states_0(12);

alpha0 = theta0;
beta0 = 0;
wdot0 = 0;

run('validation_helper.m')

%% Function to correct lateral stability derivatives
function SD_Lateral_Final = Lateral_correction(SD_Lat_dash, Vto, Ix, Iz, Ixz)
    
    G = 1 / (1 - (Ixz^2 / (Ix * Iz)));
%%% Drivatives dashed 
  Yv = SD_Lat_dash(1);
  Yb = SD_Lat_dash(2);
  L_beta_dash = SD_Lat_dash(3);
  N_beta_dash = SD_Lat_dash(4);
  L_p_dash    = SD_Lat_dash(5);
  N_p_dash    = SD_Lat_dash(6);
  L_r_dash    = SD_Lat_dash(7);
  N_r_dash    = SD_Lat_dash(8);
  Y_star_da   = SD_Lat_dash(9);
  Y_star_dr   = SD_Lat_dash(10);
  L_da_dash   = SD_Lat_dash(11);
  N_da_dash   = SD_Lat_dash(12);
   L_dr_dash  = SD_Lat_dash(13);
  N_dr_dash   = SD_Lat_dash(14);
  %%% undashed drivatives
  M =([G Ixz*G/Ix ;Ixz*G/Iz G]); %% the matrix used to transfer from dashed to undashed
  M_inv = inv(M) ;
  %%%
  A = M_inv * [ L_beta_dash ; N_beta_dash ] ;
  L_beta = A(1);
  N_beta = A(2);
  %%%
 B = M_inv * [L_p_dash ;  N_p_dash  ] ;
  L_p = B(1);
  N_p = B(2); 
  %%%
  C = M_inv * [L_r_dash ;  N_r_dash  ] ;
  L_r = C(1);
  N_r = C(2);
  %%%
  D = M_inv*[L_da_dash ; N_da_dash ] ;  %%% A B C D E just symboles
  L_da = D(1);
  N_da = D(2) ;
  %%%
  E = M_inv*[L_dr_dash ; N_dr_dash ] ;
  L_dr = E(1);
  N_dr = E(2);
  %%%
  Y_da = Y_star_da*Vto ;
  %%%
  Y_dr   = Y_star_dr *Vto ;
  
%%%%
    SD_Lateral_Final = [Yv, Yb ,L_beta, N_beta ,L_p ,N_p , L_r ,N_r,Y_da , Y_dr, L_da , N_da , L_dr , N_dr , ];
end

%% Runge-Kutta 4th Order Integration
function [t_vec, states_vec, delta_States] = raunge_kutta_4(t_vec, states_0, Forces, Moments, mass, Inertia)
    n = length(t_vec);
    dt = t_vec(2) - t_vec(1);
    states_vec = zeros(12, n);
    states_vec(:, 1) = states_0;

    for i = 1:n-1
        K1 = get_states_dot(t_vec(i), states_vec(:, i), Forces(:, i), Moments(:, i), mass, Inertia);
        K2 = get_states_dot(t_vec(i) + 0.5 * dt, states_vec(:, i) + K1 * 0.5 * dt, Forces(:, i), Moments(:, i), mass, Inertia);
        K3 = get_states_dot(t_vec(i) + 0.5 * dt, states_vec(:, i) + K2 * 0.5 * dt, Forces(:, i), Moments(:, i), mass, Inertia);
        K4 = get_states_dot(t_vec(i) + dt, states_vec(:, i) + K3 * dt, Forces(:, i), Moments(:, i), mass, Inertia);

        states_vec(:, i+1) = states_vec(:, i) + dt / 6 * (K1 + 2 * K2 + 2 * K3 + K4);
    end

    delta_States = states_vec - states_0;
end

%% Function to compute state derivatives
function states_dot = get_states_dot(t_vec, states_vec, Forces, Moments, m, I_mat)
    % Extract states
    u = states_vec(1);
    v = states_vec(2);
    w = states_vec(3);
    p = states_vec(4);
    q = states_vec(5);
    r = states_vec(6);
    phi = states_vec(7);
    theta = states_vec(8);
    psi = states_vec(9);
    x = states_vec(10);
    y = states_vec(11);
    z = states_vec(12);

    % Rotation matrix for Euler angles (ZYX convention)
    J = [1, sin(phi) * tan(theta), cos(phi) * tan(theta);
         0, cos(phi), -sin(phi);
         0, sin(phi) / cos(theta), cos(phi) / cos(theta)];

    % Compute translational dynamics
    vel_dot = (1 / m) * Forces - cross([p; q; r], [u; v; w]);

    % Compute rotational dynamics
    omega_dot = I_mat \ (Moments - cross([p; q; r], I_mat * [p; q; r]));

    % Compute Euler angle rates
    euler_dot = J * [p; q; r];

    % Compute position rates in the inertial frame
    pos_dot = eul2rotm([psi, theta, phi], 'ZYX') * [u; v; w];

    % Concatenate state derivatives
    states_dot = [vel_dot; omega_dot; euler_dot; pos_dot];
end

%% Function to compute forces and moments
function [Forces, Moments] = get_forces_and_moments(delta_states, M_0, Matrix_states, Matrix_controls, delta_control)
    % Compute forces and moments based on state perturbations and control inputs
    delta_FM = Matrix_states * delta_states + Matrix_controls * delta_control';
    Forces = delta_FM(1:3);
    Moments = delta_FM(4:6) + M_0;
end