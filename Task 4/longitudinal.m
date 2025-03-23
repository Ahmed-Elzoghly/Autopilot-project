
%% Linearized_Matrices
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
% elevator TF 
u_de = Long_TF(1,1);
w_de = Long_TF(2,1);
q_de = Long_TF(3,1);
theta_de = Long_TF(4,1);
% Thrustor
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
%% root locus for short mode
%%%%% with delta elevator
figure
subplot(1,2,1);
rlocus(w_de);
title('(w_de) Linear Full Model', 'Interpreter', 'none');
subplot(1,2,2);
rlocus(w_de_sp);
title('(w_de_sp) short period mode','Interpreter', 'none');
figure
subplot(1,2,1);
rlocus(q_de);
title('(q_de) Linear Full Model', 'Interpreter', 'none');
subplot(1,2,2);
rlocus(q_de_sp);
title('(q_de_sp) short period mode','Interpreter', 'none');
%%%%% with delta thrust
figure
subplot(1,2,1);
rlocus(w_dth);
title('(w_dth) Linear Full Model', 'Interpreter', 'none');
subplot(1,2,2);
rlocus(w_dth_sp);
title('(w_dth_sp) short period mode','Interpreter', 'none');
figure
subplot(1,2,1);
rlocus(q_dth);
title('(q_dth) Linear Full Model', 'Interpreter', 'none');
subplot(1,2,2);
rlocus(q_dth_sp);
title('(q_dth_sp) short period mode','Interpreter', 'none');

%% root locus for long mode
%%%%% with delta elevator
figure
subplot(1,2,1);
rlocus(u_de);
title('(u_de) Linear Full Model', 'Interpreter', 'none');
subplot(1,2,2);
rlocus(u_de_lp);
title('(u_de_lp) long period mode','Interpreter', 'none');
figure
subplot(1,2,1);
rlocus(theta_de);
title('(theta_de) Linear Full Model', 'Interpreter', 'none');
subplot(1,2,2);
rlocus(theta_de_lp);
title('(theta_de_lp) long period mode','Interpreter', 'none');
%%%%% with delta thrust
figure
subplot(1,2,1);
rlocus(u_dth);
title('(u_dth) Linear Full Model', 'Interpreter', 'none');
subplot(1,2,2);
rlocus(u_de_lp);
title('(u_dth_lp) long period mode','Interpreter', 'none');
figure
subplot(1,2,1);
rlocus(theta_dth);
title('(theta_dth) Linear Full Model', 'Interpreter', 'none');
subplot(1,2,2);
rlocus(theta_dth_lp);
title('(theta_dth_lp) long period mode','Interpreter', 'none');
