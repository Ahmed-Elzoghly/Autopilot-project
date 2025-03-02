clc; clearvars; close all;

%% Select Plane and Flight Condition
addpath('Planes_Data');
addpath('Validation');
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

%% Forces and moments
[Forces, Moments] = get_forces_and_moments(delta_states@raunge_kutta_4() , M_0, Matrix_states, Matrix_controls, delta_control)


%% Extract variables for validation_helper.m
% Extract position coordinates (x, y, z)
x = states_vec_RK4(10, :); % x position
y = states_vec_RK4(11, :); % y position
z = states_vec_RK4(12, :); % z position

% Extract velocity components (u, v, w)
u = states_vec_RK4(1, :); % u velocity
v = states_vec_RK4(2, :); % v velocity
w = states_vec_RK4(3, :); % w velocity

% Extract angular rates (p, q, r) and convert to degrees
p_deg = rad2deg(states_vec_RK4(4, :)); % p in deg/sec
q_deg = rad2deg(states_vec_RK4(5, :)); % q in deg/sec
r_deg = rad2deg(states_vec_RK4(6, :)); % r in deg/sec

% Extract Euler angles (phi, theta, psi) and convert to degrees
phi_deg = rad2deg(states_vec_RK4(7, :)); % phi in deg
theta_deg = rad2deg(states_vec_RK4(8, :)); % theta in deg
psi_deg = rad2deg(states_vec_RK4(9, :)); % psi in deg

% Compute alpha (angle of attack) and beta (sideslip angle)
alpha_deg = rad2deg(atan2(w, u)); % alpha in deg
beta_deg = rad2deg(atan2(v, sqrt(u.^2 + w.^2))); % beta in deg

% Position matrix P
P = [x; y; z];

% Time vector for plotting
time_V = time_vec;

%% Call validation_helper.m
run('validation_helper.m');


%% Function to compute forces and moments
function [Forces, Moments] = Get_Forces_Moments(delta_control, States_0, ...
                          Forces_0, m, g, ...
                          I_mat, States, Forces, Vto)

    % Initialize
    Vb_dot_0 = [0; 0; 0];
    Vb_0 = States_0(1:3);
    Vomega_0 = States_0(4:6);

    Vb = States(1:3);
    Vomega = States(4:6);
    Vb_dot = Forces ./ m - cross(Vomega, Vb);

    delta_States = States - States_0;

    % Extract linear & angular velocity changes
    delta_u = delta_States(1);    delta_v = delta_States(2);    delta_w = delta_States(3);
    delta_p = delta_States(4);    delta_q = delta_States(5);    delta_r = delta_States(6);

    % Body axis velocities
    u = Vb(1);    v = Vb(2);    w = Vb(3);
    u0 = Vb_0(1); v0 = Vb_0(2); w0 = Vb_0(3);

    % Initial total airspeed
    Vt_0 = sqrt(u0^2 + v0^2 + w0^2);

    % Body axis accelerations
    u_dot = Vb_dot(1);    v_dot = Vb_dot(2);    w_dot = Vb_dot(3);
    u_dot0 = Vb_dot_0(1); v_dot0 = Vb_dot_0(2); w_dot0 = Vb_dot_0(3);

    % Body axis angular rates
    p = Vomega(1);    q = Vomega(2);    r = Vomega(3);
    p0 = Vomega_0(1); q0 = Vomega_0(2); r0 = Vomega_0(3);

  

    % Derived lateral derivatives (normalized)
    Lv = LB / Vto;  Nv = NB / Vto;  Yv = YB / Vto;
    Yp = 0; Yr = 0;  % Defaults if not provided

    % Control surface deflections
    Da = delta_control(1);  Dr = delta_control(2);
    De = delta_control(3);  Dth = delta_control(4);

 

 

    % Combined state change vector (including rotational acceleration)
    delta_States_vec = [delta_States(1:6); w_dot - w_dot0];

    % Compute force and moment changes from state and control changes
    delta_FM = States_Matrix * delta_States_vec + Controls_Matrix * delta_control';

    % Mass moment of inertia vector
    I_vec = diag(I_mat);

    % Compute changes in forces and moments
    delta_Forces = m * delta_FM(1:3);
    delta_Moments = I_vec .* delta_FM(4:6);

    % Compute gravitational forces in body frame
    phi = States(7);  theta = States(8);  psi = States(9);
    Forces_Inertia = eul2rotm([psi, theta, phi], 'ZYX')' * [0; 0; m*g];

    % Total forces and moments
    Forces = delta_Forces + Forces_0 + Forces_Inertia;
    Moments = delta_Moments + Moments_0;

end
