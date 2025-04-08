function Task_4_longitudinal()
%{
Longitudinal Dynamics Analysis Tool
Inputs: Aircraft data file (Excel)
Outputs: Stability mode plots and frequency responses
Required Toolboxes: Control System, Statistics
%}

%% Initialize Environment
clc; clear; close all;

%% ------------------
%% Data Validation
%% ------------------
Plane_data_file = 'Jetstar_FC10.xlsx'; % Explicit extension

% Check file existence
if ~exist(Plane_data_file, 'file')
    error('Aircraft data file "%s" not found!', Plane_data_file);
end

% Read data with dimension check
aircraft_data = xlsread(Plane_data_file, 'B2:B61');
if numel(aircraft_data) < 60
    error('Insufficient data in aircraft_data vector');
end

%% ------------------
%% Aircraft Initialization
%% ------------------
%% Time vector
dt = aircraft_data(1);
tfinal = aircraft_data(2); 
t_vec = 0:dt:tfinal;
n = tfinal/dt;

%% Control inputs
delta_aileron = aircraft_data(57);
delta_rudder = aircraft_data(58);
delta_elevator = aircraft_data(59);
delta_thrust = aircraft_data(60); % In lbs

% Convert thrust to Newtons (1 lb = 4.44822 N)
delta_thrust = delta_thrust * 4.44822; 
delta_control = [deg2rad([delta_aileron, delta_rudder, delta_elevator]), delta_thrust];

%% Mass & Inertia
mass = aircraft_data(51);
gravity = aircraft_data(52);

Ixx = aircraft_data(53);
Iyy = aircraft_data(54);
Izz = aircraft_data(55);
Ixz = aircraft_data(56);
Inertia = [Ixx, 0, -Ixz;
           0, Iyy, 0;
           -Ixz, 0, Izz];

% Safer matrix inversion
if cond(Inertia) > 1e12
    error('Near-singular inertia matrix detected');
end
I_inv = inv(Inertia);

%% ------------------
%% State-Space Modeling
%% ------------------
%% Initial conditions
states_0 = aircraft_data(4:15);
Vtotal_0 = norm(states_0(1:3));

%% Stability derivatives
SD_long = aircraft_data(21:36);
[Xu, Zu, Mu, Xw, Zw, Mw, Zwd, Zq, Mwd, Mq, Xde, Zde, Mde, Xdth, Zdth, Mdth] = deal(SD_long(:));

% Lateral derivatives (assuming Lateral_correction is defined elsewhere)
SD_lat_dash = aircraft_data(37:50);
SD_lateral_final = Lateral_correction(SD_lat_dash, Vtotal_0, Ixx, Izz, Ixz);
[Yv, Yb, Lb, Nb, Lp, Np, Lr, Nr, Yda, Ydr, Lda, Nda, Ldr, Ndr] = deal(SD_lateral_final(:));

%% Longitudinal State-Space
u0 = states_0(1);
w0 = states_0(3);
theta0 = states_0(8);
g = gravity;
Z_denominator = 1/(1-Zwd); % Precompute

A_long = [Xu,      Xw,          -w0,        -g*cos(theta0);
          Zu*Z_denominator, Zw*Z_denominator, (Zq+u0)*Z_denominator, -g*sin(theta0)*Z_denominator;
          Mu+Mwd*Zu*Z_denominator, Mw+Mwd*Zw*Z_denominator, Mq+Mwd*(Zq+u0)*Z_denominator, -Mwd*g*sin(theta0)*Z_denominator;
          0,       0,           1,           0];

B_long = [Xde,       Xdth*4.44822; % Thrust converted to N
          Zde*Z_denominator, Zdth*Z_denominator*4.44822;
          Mde+Mwd*Zde*Z_denominator, Mdth*4.44822+Mwd*Zdth*Z_denominator*4.44822;
          0,       0];

%% Model Validation
Eigenvalues = eig(A_long);
if any(real(Eigenvalues) > 0)
    warning('Unstable system detected!');
end

%% ------------------
%% Simulation & Analysis
%% ------------------
%% Step Response Configuration
elevator_inputs = deg2rad([1, 5, 25]);
thrust_inputs = [2000, 10000] * 4.44822; % Convert lbs to N
T_final = 1200;
t = linspace(0, T_final, 1000);

%% Generate Plots
plot_responses(A_long, B_long, elevator_inputs, thrust_inputs, t);

%% Stability Analysis
plot_stability_analysis(A_long, B_long);

end

%% ------------------
%% Plotting Utilities
%% ------------------
function plot_responses(A_long, B_long, elevator_inputs, thrust_inputs, t)
% Full state-space
C_long = eye(4);
D_long = zeros(4,2);
SS_Long = ss(A_long, B_long, C_long, D_long);

% Short-period approximation
A_sp = A_long(2:3, 2:3);
B_sp = B_long(2:3, :);
SS_sp = ss(A_sp, B_sp, eye(2), zeros(2,2));

% Phugoid approximation
A_ph = [A_long(1,1), A_long(1,4);
        -B_long(2,1)/(A_long(2,3)), 0]; % Simplified phugoid model
B_ph = B_long([1,4], :);
SS_ph = ss(A_ph, B_ph, eye(2), zeros(2,2));

%% Elevator Response
for j = 1:length(elevator_inputs)
    [y_full, ~] = step(SS_Long(:,1)*elevator_inputs(j), t);
    [y_sp, ~] = step(SS_sp(:,1)*elevator_inputs(j), t);
    [y_ph, ~] = step(SS_ph(:,1)*elevator_inputs(j), t);
    
    create_figure(y_full, y_sp, y_ph, t, ...
        sprintf('Elevator Input: %d°', rad2deg(elevator_inputs(j))), ...
        'e');
end

%% Thrust Response
for j = 1:length(thrust_inputs)
    [y_full, ~] = step(SS_Long(:,2)*thrust_inputs(j), t);
    [y_sp, ~] = step(SS_sp(:,2)*thrust_inputs(j), t);
    [y_ph, ~] = step(SS_ph(:,2)*thrust_inputs(j), t);
    
    create_figure(y_full, y_sp, y_ph, t, ...
        sprintf('Thrust Input: %.0f N', thrust_inputs(j)), ...
        'T');
end
end

function create_figure(y_full, y_sp, y_ph, t, input_label, input_type)
figure('Position', [100 100 1000 800]);

% Short Period Plots
subplot(2,2,1);
plot(t, y_full(:,2), 'b', t, y_sp(:,1), 'r--', 'LineWidth', 1.5);
title('Vertical Velocity (w)'); 
xlabel('Time (s)'); ylabel('m/s'); 
legend('Full Model', 'Short Period', 'Location','best');
grid on;

subplot(2,2,2);
plot(t, rad2deg(y_full(:,3)), 'b', t, rad2deg(y_sp(:,2)), 'r--', 'LineWidth', 1.5);
title('Pitch Rate (q)'); 
xlabel('Time (s)'); ylabel('deg/s'); 
grid on;

% Phugoid Plots
subplot(2,2,3);
plot(t, y_full(:,1), 'b', t, y_ph(:,1), 'g--', 'LineWidth', 1.5);
title('Longitudinal Velocity (u)'); 
xlabel('Time (s)'); ylabel('m/s'); 
legend('Full Model', 'Phugoid', 'Location','best');
grid on;

subplot(2,2,4);
plot(t, rad2deg(y_full(:,4)), 'b', t, rad2deg(y_ph(:,2)), 'g--', 'LineWidth', 1.5);
title('Pitch Angle (\theta)'); 
xlabel('Time (s)'); ylabel('deg'); 
grid on;

% Add unified title
if exist('sgtitle', 'file')
    sgtitle(sprintf('Longitudinal Response - %s', input_label));
else
    suptitle(sprintf('Longitudinal Response - %s', input_label));
end
end

function plot_stability_analysis(A_long, B_long)
%% Root Locus and Bode Plots
SS_Long = ss(A_long, B_long, eye(4), zeros(4,2));
tf_full = tf(SS_Long);

% Configure plots
figure('Position', [100 100 1200 800]);
subplot(2,2,1);
rlocus(tf_full(2,1)); % w/de
title('Root Locus: w/\delta_e');

subplot(2,2,2);
rlocus(tf_full(3,1)); % q/de
title('Root Locus: q/\delta_e');

subplot(2,2,3);
bode(tf_full(2,1));   % w/de
title('Bode: w/\delta_e');

subplot(2,2,4);
bode(tf_full(3,1));   % q/de
title('Bode: q/\delta_e');

sgtitle('Stability Analysis - Elevator Input');
end