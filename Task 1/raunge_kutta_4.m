function [t_vec, y] = raunge_kutta_4(f_vec, t_vec, y0)

%% RK4
n = length(t_vec);
dt = t_vec(2) - t_vec(1);
y(:,1) =y0;
for i = 1:n
   K1 = f_vec(t_vec(i) , y(:,i));
   K2 = f_vec(t_vec(i)+dt/2 , y(:,i)+K1*dt/2);
   K3 = f_vec(t_vec(i)+dt/2 , y(:,i)+K2*dt/2);
   K4 = f_vec(t_vec(i)+dt , y(:,i)+K3*dt);
    
    y(:,i+1) = y(:,i)+dt/6*(K1+2*K2+2*K3+K4);
    
end
y = y(: , 1:n);

end