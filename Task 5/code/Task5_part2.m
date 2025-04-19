clc; clearvars; close all;
run("Calculations.m");
%% linearization
%[A, B, C, D] = linmod('Simulator21', [u0;v0;w0;p0;q0;r0;phi0;theta0;psi0]);
%% Longitudinal State-Space Model

% Define gravity
g = gravity;

% Compute denominator for repeated terms
den_Zwd = 1 - Zwd;

% State-space matrices
A_long = [Xu,                    Xw,                    -w0,                         -g*cos(theta0); ...
          Zu/den_Zwd,            Zw/den_Zwd,            (Zq+u0)/den_Zwd,             -g*sin(theta0)/den_Zwd; ...
          Mu + Mwd*Zu/den_Zwd,   Mw + Mwd*Zw/den_Zwd,    Mq + Mwd*(Zq+u0)/den_Zwd,   -Mwd*g*sin(theta0)/den_Zwd; ...
          0,                     0,                     1,                           0];

B_long = [Xde,                       Xdth; ...
          Zde/den_Zwd,               Zdth/den_Zwd; ...
          Mde + Mwd*Zde/den_Zwd,     Mdth + Mwd*Zdth/den_Zwd; ...
          0,                         0];

C_long = eye(4);
D_long = zeros(4,2);

% Create state-space model
Long_ss = ss(A_long, B_long, C_long, D_long);
Long_TF = tf(Long_ss);
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
%% control transfer function 
servo            = tf(10,[1 10]);
integrator       = tf(1,[1 0]);
differentiator   = tf([1 0],1);
engine_timelag   = tf(0.1,[1 0.1]);

%% Velocity control 
OL_u_ucom = servo*engine_timelag*u_dth;
