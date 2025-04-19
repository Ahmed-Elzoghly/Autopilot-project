function states_dot = RBD_Model(t,states_vec,F,M,I,Mass)
  
u = states_vec(1);
v = states_vec(2);
w = states_vec(3);

p = states_vec(4);
q = states_vec(5);
r = states_vec(6);

phi = states_vec(7);
theta = states_vec(8);
psi = states_vec(9);

x = states_vec(10);
y = states_vec(11);
z = states_vec(12);

states_dot(1:3,1)   = 1/Mass.*F - cross([p;q;r],[u;v;w]); 
states_dot(4:6,1)   = inv(I)*(M-cross([p;q;r],I*[p;q;r]));
states_dot(7:9,1)   = [1 ,sin(phi)*tan(theta),cos(phi)*tan(theta);...
                       0,cos(phi),-sin(phi);...
                       0,sin(phi)/cos(theta),cos(phi)/cos(theta)] * [p;q;r];
states_dot(10:12,1) = eul2rotm([psi theta phi]) * [u;v;w];

end