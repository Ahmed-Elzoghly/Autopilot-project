clc
close all
clearvars 
%% Problem definition
y(:,1) = [2;1];
t0 = 0 ;
tf = 20 ;
n = 100 ;
f_vec = @(t,y_vec)[sin(t)+cos(y_vec(1))+cos(y_vec(2)) ; sin(t)+sin(y_vec(2))];

%% Time vector creation

dt = (tf-t0)/n;
t_vec = (0:1:n-1)*dt;
%% RK4

[t_vec_rk4,y_rk4] = raunge_kutta_4(f_vec ,t_vec,y(:,1));
%% ODE45

[t_vec_ode45,y_ode45] = ode45(f_vec ,t_vec,y(:,1));

%% Simulink RK4

out = sim("RK4_simulink.slx");

%% plot and compare 
figure 
plot(t_vec_rk4,y_rk4(1,:))
hold on ; 
plot(t_vec,y_ode45(:,1));
plot(out.tout,out.y_simulink(:,1));
grid on;
legend(["RK4" , "ODE45", "RK4 simulink" ])
title ("Y_1")


figure  
plot(t_vec_rk4,y_rk4(2,:))
hold on ; 
plot(t_vec,y_ode45(:,2));
plot(out.tout,out.y_simulink(:,2));
grid on ; 
legend(["RK4" , "ODE45", "RK4 simulink" ])
title ("Y_2")