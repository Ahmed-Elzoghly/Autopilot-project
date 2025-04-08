clc; clearvars; close all;

%% Read Data
% Select the aircraft data file
Plane_data_file = 'Jetstar_FC10';

% Read aircraft parameters from Excel file
aircraft_data = xlsread(Plane_data_file, 'B2:B61');

%% Time Vector Initialization
dt = aircraft_data(1);  % Time step
tfinal = aircraft_data(2); % Final simulation time
t_vec = 0:dt:tfinal;
n = tfinal/dt;

%% Control Inputs
delta_control = deg2rad([aircraft_data(57), ... % Aileron
                         aircraft_data(58), ... % Rudder
                         aircraft_data(59)])';  % Elevator
delta_thrust = aircraft_data(60);

%% Aircraft Properties
mass = aircraft_data(51);
gravity = aircraft_data(52);

% Inertia Tensor Components
Ixx = aircraft_data(53); Iyy = aircraft_data(54);
Izz = aircraft_data(55); Ixz = aircraft_data(56);
Inertia = [Ixx, 0, -Ixz; 0, Iyy, 0; -Ixz, 0, Izz];
I_inv = inv(Inertia);

%% Initial Conditions
states_0 = aircraft_data(4:15);
Vtotal_0 = norm(states_0(1:3)); % Total velocity

% Compute Initial Gravity Forces
F_gravity_0 = mass * gravity * [sin(states_0(8)); ...
                               -cos(states_0(8)) * sin(states_0(7)); ...
                               -cos(states_0(8)) * cos(states_0(7))];

%% Stability Derivatives
SD_long = aircraft_data(21:36);
[Xu, Zu, Mu, Xw, Zw, Mw, Zwd, Zq, Mwd, Mq, ...
 Xde, Zde, Mde, Xdth, Zdth, Mdth] = num2cell(SD_long){:};

%% State Matrices for Longitudinal Motion
g = gravity;
A_long = [Xu, Xw, -Vtotal_0, -g*cos(states_0(8));
          Zu/(1-Zwd), Zw/(1-Zwd), (Zq+Vtotal_0)/(1-Zwd), -g*sin(states_0(8))/(1-Zwd);
          Mu+Mwd*Zu/(1-Zwd), Mw+Mwd*Zw/(1-Zwd), Mq+Mwd*(Zq+Vtotal_0)/(1-Zwd), -Mwd*g*sin(states_0(8))/(1-Zwd);
          0, 0, 1, 0];

B_long = [Xde, Xdth;
          Zde/(1-Zwd), Zdth/(1-Zwd);
          Mde+Mwd*Zde/(1-Zwd), Mdth+Mwd*Zdth/(1-Zwd);
          0, 0];

SS_Long = ss(A_long, B_long, eye(4), zeros(4,2));

%% Eigenvalues Analysis
eigenvalues = eig(A_long);
disp('Short Period Eigenvalues:'); disp(eigenvalues(1:2));
disp('Long Period Eigenvalues:'); disp(eigenvalues(3:4));

%% Step Response Analysis
T_final = 1200;
t = linspace(0, T_final, 1000);
elevator_inputs = deg2rad([1, 5, 25]);

for j = 1:length(elevator_inputs)
    elevator_input = elevator_inputs(j);
    SS_Long_elevator_input = SS_Long(:,1);
    [y_full, t] = step(SS_Long_elevator_input * elevator_input, t);
    
    figure;
    subplot(2,1,1);
    plot(t, y_full(:,2), 'b', 'LineWidth', 1.5);
    ylabel('w (m/s)'); grid on;
    legend('Full Response');
    
    subplot(2,1,2);
    plot(t, rad2deg(y_full(:,3)), 'b', 'LineWidth', 1.5);
    ylabel('q (deg/s)'); xlabel('Time (s)'); grid on;
    legend('Full Response');
    sgtitle(['Elevator Deflection - \delta_e = ', num2str(rad2deg(elevator_input)), '°']);
end
