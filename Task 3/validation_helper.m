% Prepared by Eng. Wessam Ahmed

figure
plot3(x,-y,-z);
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
plot(time_V,alpha_deg)
title('\alpha (deg)')
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