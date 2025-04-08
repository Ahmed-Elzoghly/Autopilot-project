clc 
clear vars 
close all 
%% Read data
%Plane_data_file = 'B747_FC5';
Plane_data_file = 'Jetstar_FC10' ;
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
% mass = weight/gravity;

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

%% Initial moments
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
%% linearization
%[A, B, C, D] = linmod('Simulator21', [u0;v0;w0;p0;q0;r0;phi0;theta0;psi0]);
%% For longitudinal 
g = gravity;
A_long=[Xu,Xw,-w0,-g*cos(theta0);...
        Zu/(1-Zwd),Zw/(1-Zwd),(Zq+u0)/(1-Zwd),-g*sin(theta0)/(1-Zwd);...
        Mu+Mwd*Zu/(1-Zwd),Mw+Mwd*Zw/(1-Zwd),Mq+Mwd*(Zq+u0)/(1-Zwd),-Mwd*g*sin(theta0)/(1-Zwd);...
        0,0,1,0];

B_long=[Xde,Xdth ;...
        Zde/(1-Zwd),Zdth/(1-Zwd);...
        Mde+Mwd*Zde/(1-Zwd),Mdth+Mwd*Zdth/(1-Zwd);...
        0,0];
C_long=eye(4);
D_long=zeros(4,2);
SS_Long = ss(A_long,B_long,C_long,D_long);    % State space for Longitudinal

%% Linear full model
Eigenvalues= eig(A_long);
[e_vecs_long,e_vals_long]= eig(A_long);
Long_ss = ss(A_long,B_long,C_long,D_long); 
Long_TF = tf(Long_ss);
Eigenvalues_sp=[Eigenvalues(1);Eigenvalues(2)];
disp('Eigenvalues_sp:') 
disp(Eigenvalues_sp)
Eigenvalues_lp=[Eigenvalues(3);Eigenvalues(4)];
disp('Eigenvalues_lp:') 
disp(Eigenvalues_lp)
% TF  with elevator 
u_de = Long_TF(1,1);
w_de = Long_TF(2,1);
q_de = Long_TF(3,1);
theta_de = Long_TF(4,1);
% TF  with Thrust
u_dth=Long_TF(1,2);
w_dth = Long_TF(2,2);
q_dth = Long_TF(3,2);
theta_dth = Long_TF(4,2);

%% Modes Approximation
% Short period
A_long_sp = [A_long(2,2),A_long(2,3);A_long(3,2),A_long(3,3)];
B_long_sp = [B_long(2,1),B_long(2,2);B_long(3,1),B_long(3,2)];
C_long_sp = eye(2);
D_long_sp = zeros(2,2);
eig_val_sp=eig(A_long_sp); %% eigen value For Approximation short period mode
disp('eig_val_sp_Approximation:') 
disp(eig_val_sp)
[vec_sp,val_sp]= eig(A_long_sp);
Long_sp_ss = ss(A_long_sp,B_long_sp,C_long_sp,D_long_sp); 
Long_sp_TF = tf(Long_sp_ss);
% elevator TF 
w_de_sp = Long_sp_TF(1,1);
q_de_sp = Long_sp_TF(2,1);
% Thrustor
w_dth_sp = Long_sp_TF(1,2);
q_dth_sp = Long_sp_TF(2,2);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% long period
A_long_lp = [A_long(1,1),A_long(1,4);-Zu/(Zq+u0) ,0];
B_long_lp = [B_long(1,1),B_long(1,2);B_long(4,1),B_long(4,2)];
C_long_lp = eye(2);
D_long_lp = zeros(2,2);
eig_val_lp = eig(A_long_lp); %% eigen value For Approximation long period mode
disp('eig_val_lp_Approximation:') 
disp(eig_val_lp)
[vec_lp,val_lp]= eig(A_long_lp);
Long_lp_ss = ss(A_long_lp,B_long_lp,C_long_lp,D_long_lp); 
Long_lp_TF = tf(Long_lp_ss);
% elevator TF 
u_de_lp = Long_lp_TF(1,1);
theta_de_lp = Long_lp_TF(2,1);
% Thrustor
u_dth_lp = Long_lp_TF(1,2);
theta_dth_lp = Long_lp_TF(2,2);
%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Step response for elevator input
elevator_inputs = deg2rad([1, 5, 25]); % different values for delta elevator 
T_final = 1200;
t = linspace(0, T_final, 1000);

for j = 1:length(elevator_inputs) 
    elevator_input = elevator_inputs(j);
    SS_Long_elevator_input = SS_Long(:,1); 

    [y_full, t] = step(SS_Long_elevator_input * elevator_input, t);
    [y_short, t] = step(Long_sp_ss(:,1) * elevator_input, t);
    [y_long, t] = step(Long_lp_ss(:,1) * elevator_input, t);

    %Figure: Short Period (w, q)
    figure;
    %%%%% w
    subplot(2,1,1);
    plot(t, y_full(:,2), 'b', 'LineWidth', 1.5); hold on; 
    plot(t, y_short(:,1), 'r-', 'LineWidth', 1.5);
    ylabel('w (m/s)'); grid on;
    legend('linear Full Response', 'Short Period Approximation');
    %%%%% q
    subplot(2,1,2);
    plot(t, rad2deg(y_full(:,3)), 'b', 'LineWidth', 1.5); hold on;
    plot(t,rad2deg(y_short(:,2)), 'r-', 'LineWidth', 1.5);
    ylabel('q (deg/s)'); xlabel('Time (seconds)'); grid on;
    legend('linear Full Response', 'Short Period Approximation');

    sgtitle(['Short Period Mode - \delta_e = ', num2str(rad2deg(elevator_input)), '°']);

    %Figure 2: Long Period (u,theta)
    figure;
    %%%%% u
    subplot(2,1,1);
    plot(t, y_full(:,1), 'b', 'LineWidth', 1.5); hold on; 
    plot(t, y_long(:,1), 'r-', 'LineWidth', 1.5);
    ylabel('u (m/s)'); grid on;
    legend('linear Full Response', 'long Period Approximation');
    %%%%% theta
    subplot(2,1,2);
    plot(t, rad2deg(y_full(:,4)), 'b', 'LineWidth', 1.5); hold on; 
    plot(t, rad2deg(y_long(:,2)), 'r-', 'LineWidth', 1.5);
    ylabel('\theta (deg)'); xlabel('Time (seconds)'); grid on;
    legend('linear Full Response', 'long Period Approximation');
    sgtitle(['Long Period Mode - \delta_e = ', num2str(rad2deg(elevator_input)), '°']);
end
%% Step response for thrust input
thrust_inputs = [2000, 10000]; % different values for delta thrust (ibs)
T_final = 1200;
t = linspace(0, T_final, 1000);

for j = 1:length(thrust_inputs) 
    thrust_input = thrust_inputs(j);
    SS_Long_thrust_input = SS_Long(:,2); 

    [y_full, t] = step(SS_Long_thrust_input * thrust_input, t);
    [y_short, ~] = step(Long_sp_ss(:,2) * thrust_input, t);
    [y_long, ~] = step(Long_lp_ss(:,2) * thrust_input, t);

    %Figure 1: Short Period (w, q)
    figure;
    %%%%% w
    subplot(2,1,1);
    plot(t, y_full(:,2), 'b', 'LineWidth', 1.5); hold on; 
    plot(t, y_short(:,1), 'r', 'LineWidth', 1.5);
    ylabel('w (m/s)'); grid on;
    legend('linear Full Response', 'Short Period Approximation');
    %%%%% q
    subplot(2,1,2);
    plot(t, rad2deg(y_full(:,3)), 'b', 'LineWidth', 1.5); hold on; 
    plot(t, rad2deg(y_short(:,2)), 'r', 'LineWidth', 1.5);
    ylabel('q (deg/s)'); xlabel('Time (seconds)'); grid on;
    legend('linear Full Response', 'Short Period Approximation');

    sgtitle(['Short Period Mode - \delta_T = ', num2str(thrust_input), ' lbs']);

    %Figure 2: Long Period (u, \theta)
    %%%%% u
    figure;
    subplot(2,1,1);
    plot(t, y_full(:,1), 'b', 'LineWidth', 1.5); hold on; 
    plot(t, y_long(:,1), 'r', 'LineWidth', 1.5);
    ylabel('u (m/s)'); grid on;
    legend('linear Full Response', 'long Period Approximation');
    %%%%% theta
    subplot(2,1,2);
    plot(t, rad2deg(y_full(:,4)), 'b', 'LineWidth', 1.5); hold on; 
    plot(t, rad2deg(y_long(:,2)), 'r', 'LineWidth', 1.5);
    ylabel('\theta (deg)'); xlabel('Time (seconds)'); grid on;
    legend('linear Full Response', 'long Period Approximation');

    sgtitle(['Long Period Mode - \delta_T = ', num2str(thrust_input), ' lbs']);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Root locus
%%% Root locus for short period mode 
linear_full_model_short = {w_de, q_de, w_dth, q_dth};
short_period_approx = {w_de_sp, q_de_sp, w_dth_sp, q_dth_sp};
short_period_titles = {'w_de', 'q_de', 'w_dth', 'q_dth'};
for i = 1:4
    figure;
    subplot(1,2,1);
    rlocus(linear_full_model_short{i});
    title(['(', short_period_titles{i}, ') Linear Full Model'], 'Interpreter', 'none');
    
    subplot(1,2,2);
    rlocus(short_period_approx{i});
    title(['(', short_period_titles{i}, ') Short Period Mode'], 'Interpreter', 'none');
end

%%% Root locus for long period mode 
linear_full_model_long = {u_de, theta_de, u_dth, theta_dth};
long_period_approx = {u_de_lp, theta_de_lp, u_dth_lp, theta_dth_lp}; 
long_period_titles = {'u_de', 'theta_de', 'u_dth', 'theta_dth'};

for i = 1:4
    figure;
    subplot(1,2,1);
    rlocus(linear_full_model_long{i});
    title(['(', long_period_titles{i}, ') Linear Full Model'], 'Interpreter', 'none');
    
    subplot(1,2,2);
    rlocus(long_period_approx{i});
    title(['(', long_period_titles{i}, ') Long Period Mode'], 'Interpreter', 'none');
end
 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% bode plot
%%% bode plot for short period mode 
linear_full_model_short = {w_de, q_de, w_dth, q_dth};
short_period_approx = {w_de_sp, q_de_sp, w_dth_sp, q_dth_sp};
short_period_titles = {'w_de', 'q_de', 'w_dth', 'q_dth'};
for i = 1:4
    figure;
    subplot(1,2,1);
    bode(linear_full_model_short{i});
    title(['(', short_period_titles{i}, ') Linear Full Model'], 'Interpreter', 'none');
    
    subplot(1,2,2);
    bode(short_period_approx{i});
    title(['(', short_period_titles{i}, ') Short Period Mode'], 'Interpreter', 'none');
end

%%% bode plot for long period mode 
linear_full_model_long = {u_de, theta_de, u_dth, theta_dth};
long_period_approx = {u_de_lp, theta_de_lp, u_de_lp, theta_dth_lp}; 
long_period_titles = {'u_de', 'theta_de', 'u_dth', 'theta_dth'};

for i = 1:4
    figure;
    subplot(1,2,1);
    bode(linear_full_model_long{i});
    title(['(', long_period_titles{i}, ') Linear Full Model'], 'Interpreter', 'none');
    
    subplot(1,2,2);
    bode(long_period_approx{i});
    title(['(', long_period_titles{i}, ') Long Period Mode'], 'Interpreter', 'none');
end

