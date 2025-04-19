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
SS_Long = ss(A_long, B_long, C_long, D_long);

%% Linear full model
Eigenvalues= eig(A_long);
[e_vecs_long,e_vals_long]= eig(A_long);
Long_ss = ss(A_long,B_long,C_long,D_long); 
Long_TF = tf(Long_ss);
Eigenvalues_sp=[Eigenvalues(1);Eigenvalues(2)];
%disp('Eigenvalues_sp:') 
%disp(Eigenvalues_sp)
Eigenvalues_lp=[Eigenvalues(3);Eigenvalues(4)];
%disp('Eigenvalues_lp:') 
%disp(Eigenvalues_lp)
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
%disp('eig_val_sp_Approximation:') 
%disp(eig_val_sp)
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
A_long_lp = [Xu+w0*Zu/(Zq+u0),-g*cos(theta0)-w0*g*sin(theta0)/(Zq+u0);-Zu/(Zq+u0) ,g*sin(theta0)/(Zq+u0)];
B_long_lp = [Xde+w0*Zde/(Zq+u0),Xdth+w0*Zdth/(Zq+u0);-Zde/(Zq+u0),-Zdth/(Zq+u0)];
C_long_lp = eye(2);
D_long_lp = zeros(2,2);
eig_val_lp = eig(A_long_lp); %% eigen value For Approximation long period mode
%disp('eig_val_lp_Approximation:') 
%disp(eig_val_lp)
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

% %% Step response for elevator input
% elevator_inputs = deg2rad([1, 5, 25]); % different values for delta elevator 
% T_final = 300;
% t = linspace(0, T_final, 1000);
% filename = 'figures\Longitudinal Results\response\delta_e';
% 
% delta_aileron = 0;
% delta_rudder = 0;
% delta_elevator =0;
% delta_thrust = 0;
% 
% for j = 1:length(elevator_inputs) 
%     elevator_input = elevator_inputs(j);
%     SS_Long_elevator_input = SS_Long(:,1);
% 
% 
%     delta_elevator = elevator_input;
%     simOut = sim('Simulator21.slx');
% 
%     simData = {simOut.u_simulink, simOut.v_simulink, simOut.w_simulink, ...
%                    simOut.p_simulink, simOut.q_simulink, simOut.r_simulink, ...
%                    simOut.phi_simulink, simOut.theta_simulink, simOut.psi_simulink, ...
%                    simOut.x_simulink, simOut.y_simulink, simOut.z_simulink};
% 
%     [y_full, t] = step(SS_Long_elevator_input * elevator_input, t);
%     [y_short, t] = step(Long_sp_ss(:,1) * elevator_input, t);
%     [y_long, t] = step(Long_lp_ss(:,1) * elevator_input, t);
% 
%     %Figure: Short Period (w, q)
%     figure('units','normalized','outerposition',[0 0 1 1])
%     %%%%% w
%     subplot(2,1,1);
%     plot(t, y_full(:,2)+w0, 'b', 'LineWidth', 1.5); hold on; 
%     plot(t, y_short(:,1)+w0, 'g-', 'LineWidth', 1.5); hold on;
% 
%     plot(simData{3}.time, simData{3}.data, '--r', 'LineWidth', 1.5);
% 
%     ylabel('w (m/s)'); grid on;
%     legend('linear Full Response', 'Short Period Approximation', 'Non Linear');
%     %%%%% q
%     subplot(2,1,2);
%     plot(t, rad2deg(y_full(:,3))+rad2deg(q0), 'b', 'LineWidth', 1.5); hold on;
%     plot(t,rad2deg(y_short(:,2))+rad2deg(q0), 'g-', 'LineWidth', 1.5); hold on;
%     plot(simData{5}.time, simData{5}.data, '--r', 'LineWidth', 1.5);
%     ylabel('q (deg/s)'); xlabel('Time (seconds)'); grid on;
%     legend('linear Full Response', 'Short Period Approximation', 'Non Linear');
% 
%     sgtitle(['Short Period Mode - \delta_e = ', num2str(rad2deg(elevator_input)), '°']);
% 
%     set(findall(gcf,'type','line'),'linewidth',1.7);grid on ;legend ;
%        saveas(gcf,fullfile(filename,strcat('wAndq_due_d_e = ',num2str(rad2deg(elevator_input)),'.png')));
% 
% 
% 
%     %Figure 2: Long Period (u,theta)
%     figure('units','normalized','outerposition',[0 0 1 1])
%     %%%%% u
%     subplot(2,1,1);
%     plot(t, y_full(:,1)+u0, 'b', 'LineWidth', 1.5); hold on; 
%     plot(t, y_long(:,1)+u0, 'g-', 'LineWidth', 1.5); hold on;
%     plot(simData{1}.time, simData{1}.data, '--r', 'LineWidth', 1.5);
%     ylabel('u (m/s)'); grid on;
%     legend('linear Full Response', 'long Period Approximation', 'Non Linear');
%     %%%%% theta
%     subplot(2,1,2);
%     plot(t, rad2deg(y_full(:,4))+rad2deg(theta0), 'b', 'LineWidth', 1.5); hold on; 
%     plot(t, rad2deg(y_long(:,2))+rad2deg(theta0), 'g-', 'LineWidth', 1.5); hold on;
%     plot(simData{8}.time, simData{8}.data, '--r', 'LineWidth', 1.5);
%     ylabel('{\theta} (deg)'); xlabel('Time (seconds)'); grid on;
%     legend('linear Full Response', 'long Period Approximation','Non Linear');
%     sgtitle(['Long Period Mode - \delta_e = ', num2str(rad2deg(elevator_input)), '°']);
% 
%     set(findall(gcf,'type','line'),'linewidth',1.7);grid on ;legend ;
%     saveas(gcf,fullfile(filename,strcat('uAndTheta_due_d_e = ',num2str(rad2deg(elevator_input)),'.png')));
% 
% end
% %% Step response for thrust input
% thrust_inputs = [2000, 10000]; % different values for delta thrust (ibs)
% t = linspace(0, T_final, 1000);
% filename = 'figures\Longitudinal Results\response\delta_th';
% delta_aileron = 0;
% delta_rudder = 0;
% delta_elevator =0;
% delta_thrust = 0;
% for j = 1:length(thrust_inputs) 
%     thrust_input = thrust_inputs(j);
%     SS_Long_thrust_input = SS_Long(:,2); 
% 
%     delta_thrust = thrust_input;
% 
%     simOut = sim('Simulator21.slx');
% 
%     simData = {simOut.u_simulink, simOut.v_simulink, simOut.w_simulink, ...
%                    simOut.p_simulink, simOut.q_simulink, simOut.r_simulink, ...
%                    simOut.phi_simulink, simOut.theta_simulink, simOut.psi_simulink, ...
%                    simOut.x_simulink, simOut.y_simulink, simOut.z_simulink};
% 
%     [y_full, t] = step(SS_Long_thrust_input * thrust_input, t);
%     [y_short, ~] = step(Long_sp_ss(:,2) * thrust_input, t);
%     [y_long, ~] = step(Long_lp_ss(:,2) * thrust_input, t);
% 
%     %Figure 1: Short Period (w, q)
%     figure('units','normalized','outerposition',[0 0 1 1])
%     %%%%% w
%     subplot(2,1,1);
%     plot(t, y_full(:,2)+w0, 'b', 'LineWidth', 1.5); hold on; 
%     plot(t, y_short(:,1)+w0, 'g', 'LineWidth', 1.5);
%     hold on;   
%     plot(simData{3}.time, simData{3}.data, '--r', 'LineWidth', 1.5);
%     legend('linear Full Response', 'long Period Approximation','Non Linear');
%     ylabel('w (m/s)'); grid on;
%     %%%%% q
%     subplot(2,1,2);
%     plot(t, rad2deg(y_full(:,3))+rad2deg(q0), 'b', 'LineWidth', 1.5); hold on; 
%     plot(t, rad2deg(y_short(:,2))+rad2deg(q0), 'g', 'LineWidth', 1.5);
%     plot(simData{5}.time, simData{5}.data, '--r', 'LineWidth', 1.5);
%     ylabel('q (deg/s)'); xlabel('Time (seconds)'); grid on;
%     legend('linear Full Response', 'Short Period Approximation', 'Non Linear');
%     sgtitle(['Short Period Mode - \delta_T = ', num2str(thrust_input), ' lbs']);
% 
% 
%     set(findall(gcf,'type','line'),'linewidth',1.7);grid on ;legend ;
%     saveas(gcf,fullfile(filename,strcat('wAndq_due_d_th = ',num2str(thrust_input),'.png')));
% 
%     %Figure 2: Long Period (u, \theta)
%     %%%%% u
%     figure('units','normalized','outerposition',[0 0 1 1])
%     subplot(2,1,1);
%     plot(t, y_full(:,1)+u0, 'b', 'LineWidth', 1.5); hold on; 
%     plot(t, y_long(:,1)+u0, 'g', 'LineWidth', 1.5);
%     plot(simData{1}.time, simData{1}.data, '--r', 'LineWidth', 1.5);
%     ylabel('u (m/s)'); grid on;
%     legend('linear Full Response', 'long Period Approximation', 'Non Linear');
%     %%%%% theta
%     subplot(2,1,2);
%     plot(t, rad2deg(y_full(:,4))+rad2deg(theta0), 'b', 'LineWidth', 1.5); hold on; 
%     plot(t, rad2deg(y_long(:,2))+rad2deg(theta0), 'g', 'LineWidth', 1.5);
%     plot(simData{8}.time, simData{8}.data, '--r', 'LineWidth', 1.5);
%     ylabel('{\theta} (deg)'); xlabel('Time (seconds)'); grid on;
%     legend('linear Full Response', 'long Period Approximation','Non Linear');
% 
%     sgtitle(['Long Period Mode - \delta_T = ', num2str(thrust_input), ' lbs']);
% 
%     set(findall(gcf,'type','line'),'linewidth',1.7);grid on ;legend ;
%     saveas(gcf,fullfile(filename,strcat('uAndTheta_due_d_th = ',num2str(thrust_input),'.png')));
% 
% end
% 
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
% %% Root locus
% %%% Root locus for short period mode 
% 
% RL_SP_filename = 'figures\Longitudinal Results\Root Locus\Short period';
% linear_full_model_short = {w_de, q_de, w_dth, q_dth};
% short_period_approx = {w_de_sp, q_de_sp, w_dth_sp, q_dth_sp};
% short_period_titles = {'w_de', 'q_de', 'w_dth', 'q_dth'};
% for i = 1:4
%     figure('units','normalized','outerposition',[0 0 1 1])
%     folderPath = fullfile(RL_SP_filename, short_period_titles{i});
% %         if ~exist(folderPath, 'dir')
% %             mkdir(folderPath);
% %         end
%     subplot(1,2,1);
%     rlocus(linear_full_model_short{i});
%     title(['(', short_period_titles{i}, ') Linear Full Model'], 'Interpreter', 'none');
%     set(findall(gcf,'type','line'),'linewidth',1.5);grid on ;
% 
%     subplot(1,2,2);
%     rlocus(short_period_approx{i});
%     title(['(', short_period_titles{i}, ') Short Period Mode'], 'Interpreter', 'none');
%     set(findall(gcf,'type','line'),'linewidth',1.5);grid on ;
% 
%     saveas(gcf,fullfile(RL_SP_filename,strcat(short_period_titles{i}, 'Full Model&Approx','.png')));
% 
% end
% 
% %%% Root locus for long period mode 
% RL_LP_filename = 'figures\Longitudinal Results\Root Locus\Long period';
% linear_full_model_long = {u_de, theta_de, u_dth, theta_dth};
% long_period_approx = {u_de_lp, theta_de_lp, u_dth_lp, theta_dth_lp}; 
% long_period_titles = {'u_de', 'theta_de', 'u_dth', 'theta_dth'};
% 
% for i = 1:4
%     figure('units','normalized','outerposition',[0 0 1 1])
%     folderPath = fullfile(RL_LP_filename, long_period_titles{i});
% %         if ~exist(folderPath, 'dir')
% %             mkdir(folderPath);
% %         end
%     subplot(1,2,1);
%     rlocus(linear_full_model_long{i});
%     title(['(', long_period_titles{i}, ') Linear Full Model'], 'Interpreter', 'none');
%     set(findall(gcf,'type','line'),'linewidth',1.5);grid on ;
% 
%     subplot(1,2,2);
%     rlocus(long_period_approx{i});
%     title(['(', long_period_titles{i}, ') Long Period Mode'], 'Interpreter', 'none');
%     set(findall(gcf,'type','line'),'linewidth',1.5);grid on ;
% 
%     saveas(gcf,fullfile(RL_LP_filename,strcat(long_period_titles{i}, 'Full Model&Approx','.png')));
% 
% end
% 
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
% %%% Bode plot for Short Period Mode
% Bode_SP_filename = 'figures\Longitudinal Results\Bode plot\Short period';
% linear_full_model_short = {w_de, q_de, w_dth, q_dth};
% short_period_approx = {w_de_sp, q_de_sp, w_dth_sp, q_dth_sp};
% short_period_titles = {'w_de', 'q_de', 'w_dth', 'q_dth'};
% 
% for i = 1:4
%     figure('units','normalized','outerposition',[0 0 1 1])
%     folderPath = fullfile(Bode_SP_filename, short_period_titles{i});
% %     if ~exist(folderPath, 'dir')
% %         mkdir(folderPath);
% %     end
% 
% 
%     bode(linear_full_model_short{i}, 'b'); hold on;   % Full Model
%     bode(short_period_approx{i}, 'r--');             % Short Period Mode 
%     hold off;
% 
%     title(['(', short_period_titles{i}, ') Bode Plot Comparison'], 'Interpreter', 'none');
%     legend('Full Model', 'Short Period Mode');  
%     set(findall(gcf,'type','line'),'linewidth',1.5);
%     grid on;
% 
%     saveas(gcf, fullfile(Bode_SP_filename, strcat(short_period_titles{i}, '_Comparison.png')));
% end
% 
% %%% Bode plot for Long Period Mode
% Bode_LP_filename = 'figures\Longitudinal Results\Bode plot\Long period';
% linear_full_model_long = {u_de, theta_de, u_dth, theta_dth};
% long_period_approx = {u_de_lp, theta_de_lp, u_de_lp, theta_dth_lp}; 
% long_period_titles = {'u_de', 'theta_de', 'u_dth', 'theta_dth'};
% 
% for i = 1:4
%     figure('units','normalized','outerposition',[0 0 1 1])
%     folderPath = fullfile(Bode_LP_filename, long_period_titles{i});
% %     if ~exist(folderPath, 'dir')
% %         mkdir(folderPath);
% %     end
% 
%     bode(linear_full_model_long{i}, 'b'); hold on;   % Full Model
%     bode(long_period_approx{i}, 'r--');             % Long Period Mode
%     hold off;
% 
%     title(['(', long_period_titles{i}, ') Bode Plot Comparison'], 'Interpreter', 'none');
%     legend('Full Model', 'Long Period Mode');  
%     set(findall(gcf,'type','line'),'linewidth',1.5);
%     grid on;
% 
%     saveas(gcf, fullfile(Bode_LP_filename, strcat(long_period_titles{i}, '_Comparison.png')));
% end
