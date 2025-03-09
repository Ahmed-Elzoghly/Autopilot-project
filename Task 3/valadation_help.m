function valadation_help(states_RK4, t_vec)

% Extract variables for validation_helper.m
% Extract position coordinates (x, y, z)
x = states_RK4(10, :); % x position
y = states_RK4(11, :); % y position
z = states_RK4(12, :); % z position

% Extract velocity components (u, v, w)
u = states_RK4(1, :); % u velocity
v = states_RK4(2, :); % v velocity
w = states_RK4(3, :); % w velocity

% Extract angular rates (p, q, r) and convert to degrees
p_deg = rad2deg(states_RK4(4, :)); % p in deg/sec
q_deg = rad2deg(states_RK4(5, :)); % q in deg/sec
r_deg = rad2deg(states_RK4(6, :)); % r in deg/sec

% Extract Euler angles (phi, theta, psi) and convert to degrees
phi_deg = rad2deg(states_RK4(7, :)); % phi in deg
theta_deg = rad2deg(states_RK4(8, :)); % theta in deg
psi_deg = rad2deg(states_RK4(9, :)); % psi in deg

% Compute alpha (angle of attack) and beta (sideslip angle)
alpha_deg = rad2deg(atan2(w, u)); % alpha in deg
beta_deg = rad2deg(atan2(v, sqrt(u.^2 + w.^2))); % beta in deg

% Position matrix P
P = [x; y; z];

% Time vector for plotting
time_V = t_vec;

% Prepared by Eng. Wessam Ahmed

figure
plot3(x,y,z);
title('Trajectory')

figure
subplot(4,3,1)
plot(time_V,u)
title('u (ft/sec)')
xlabel('time (sec)')

subplot(4,3,2)
plot(time_V,beta_deg)
title('\beta (deg)')
xlabel('time (sec)')

subplot(4,3,3)
plot(time_V,w)
title('w (ft/sec)')
xlabel('time (sec)')

subplot(4,3,4)
plot(time_V,p_deg)
title('p (deg/sec)')
xlabel('time (sec)')

subplot(4,3,5)
plot(time_V,q_deg)
title('q (deg/sec)')
xlabel('time (sec)')

subplot(4,3,6)
plot(time_V,r_deg)
title('r (deg/sec)')
xlabel('time (sec)')

subplot(4,3,7)
plot(time_V,phi_deg)
title('\phi (deg)')
xlabel('time (sec)')

subplot(4,3,8)
plot(time_V,theta_deg)
title('\theta (deg)')
xlabel('time (sec)')

subplot(4,3,9)
plot(time_V,psi_deg)
title('\psi (deg)')
xlabel('time (sec)')

subplot(4,3,10)
plot(time_V,P(1,:))
title('x (ft)')
xlabel('time (sec)')

subplot(4,3,11)
plot(time_V,P(2,:))
title('y (ft)')
xlabel('time (sec)')

subplot(4,3,12)
plot(time_V,P(3,:))
title('z (ft)')
xlabel('time (sec)')


end