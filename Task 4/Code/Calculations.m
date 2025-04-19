clc; clearvars; close all;

%% Select Plane and Flight Condition
addpath('Planes_Data');
addpath('Control design');
addpath('Validation');
Plane_data_file = 'B747_FC5';
%Plane_data_file = 'Jetstar_FC3';
% Read from the vector(aircraft_data) but take care of the order the values in excel sheet is arr
aircraft_data = xlsread(Plane_data_file, 'B2:B61'); % Read the excel sheet from B2 to B61

%% Time vector
dt = aircraft_data(1);
tfinal = aircraft_data(2); % Assuming tfinal is stored in the second entry of aircraft_data
t_vec = (0:dt:tfinal);
n = tfinal/dt;


%% Control action
delta_aileron = aircraft_data(57);
delta_rudder = aircraft_data(58);
delta_elevator = aircraft_data(59);
delta_thrust = aircraft_data(60);

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
I_inv = inv(Inertia);

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
% Initial moments
M_0 = [0; 0; 0];

%% Initialization

states_RK4 = zeros(12,n+1);
states_RK4(:,1) = states_0;
Forces = zeros(3,n+1);
Moments = zeros(3,n+1);


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
% Yv = Yb / Vtotal_0;
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
% 
% tempvariatdash = num2cell(SD_lat_dash);
% [Yv, Yb, Lbd, Nbd, Lpd, Npd, Lrd, Nrd, Yda, Ydr, Ldad, Ndad, Ldrd, Ndrd] = deal(tempvariatdash{:});
% Lvd = Lbd / Vtotal_0;
% Nvd = Nbd/Vtotal_0;
wdot = zeros(1,n+1);

% struct for delay block 

initial_states_bus.u = u0;
initial_states_bus.v = v0;
initial_states_bus.w = w0;
initial_states_bus.p = p0;
initial_states_bus.q = q0;
initial_states_bus.phi = phi0;
initial_states_bus.theta = theta0;
initial_states_bus.psi = psi0;
initial_states_bus.x = x0;
initial_states_bus.y = y0;
initial_states_bus.z = z0;
initial_states_bus.alpha = alpha0;
initial_states_bus.beta = beta0 ;
initial_states_bus.Vtotal = Vtotal_0;
initial_states_bus.wdot = wdot0;