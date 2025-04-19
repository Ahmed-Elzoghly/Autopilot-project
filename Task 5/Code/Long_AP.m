% clc; clearvars; close all;

addpath('Functions');
addpath('Sub_Scripts');
addpath('Control_Design');

run("Task_4_longtudinal.m");


% Servvo transfer function
servo = tf(10,[1 10]);
integrator = tf(1,[1 0]);
differentiarator = tf([1 0],1);
engine_timelag = tf(0.1,[1 0.1]);

%% pitch Controller 

OL_theta_thetacom = - servo * theta_de;
OL_u_ucom = servo * engine_timelag * u_dth;


% load('Long_Pitch_controller.mat');
% load('Long_velocity_controller.mat');
% load('Long_altitude_controller.mat');

load('Pich_controller.mat');

h_theta       = minreal(-integrator*(w_de/theta_de-u0));
OL_h_thetacom = tf(minreal(CL_theta_thetacom*h_theta));

%% Velocity controller