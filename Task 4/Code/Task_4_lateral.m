clc; clearvars; close all;
run("Calculations.m");
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
g = gravity;
SD_Lat_dash = aircraft_data(37:50);
Yv = SD_Lat_dash(1);
  Yb = SD_Lat_dash(2);
  L_beta_dash = SD_Lat_dash(3);
  N_beta_dash = SD_Lat_dash(4);
  L_p_dash    = SD_Lat_dash(5);
  N_p_dash    = SD_Lat_dash(6);
  L_r_dash    = SD_Lat_dash(7);
  N_r_dash    = SD_Lat_dash(8);
  Y_star_da   = SD_Lat_dash(9);
  Y_star_dr   = SD_Lat_dash(10);
  L_da_dash   = SD_Lat_dash(11);
  N_da_dash   = SD_Lat_dash(12);
   L_dr_dash  = SD_Lat_dash(13);
  N_dr_dash   = SD_Lat_dash(14);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% States = [beta ; p ; r ; phi ; psi]
A_lat = [Yv , (w0+Yp)/Vtotal_0 , (-u0+Yr)/Vtotal_0 , g*cos(theta0)/Vtotal_0 , 0;... 
        L_beta_dash , L_p_dash , L_r_dash , 0 , 0;... 
        N_beta_dash , N_p_dash , N_r_dash , 0 , 0;... 
        0 , 1 , tan(theta0) , 0 , 0;... 
        0 , 0 , 1/cos(theta0) ,0 ,0];
B_lat = [Y_star_da , Y_star_dr;... 
         L_da_dash , L_dr_dash;... 
         N_da_dash , N_dr_dash;... 
         0 , 0;... 
         0 , 0];
 C_lat=eye(5); 
 D_lat=zeros(5,2);
 lat_ss = ss(A_lat,B_lat,C_lat,D_lat); 
Eigenvalues= eig(A_lat);
%disp('Eigenvalues:') 
%disp(Eigenvalues)
[e_vecs_lat,e_vals_lat]= eig(A_lat);
lat_TF = tf(lat_ss);
% TF  with aliron
beta_da = lat_TF(1,1);
p_da = lat_TF(2,1);
r_da = lat_TF(3,1);
phi_da = lat_TF(4,1);
psi_da = lat_TF(5,1);
% TF  with rudder
beta_dr = lat_TF(1,2);
p_dr = lat_TF(2,2);
r_dr = lat_TF(3,2);
phi_dr = lat_TF(4,2);
psi_dr = lat_TF(5,2);
%% Modes Approximation
%% 3DOF mode spiral
% States [p ; r ; phi]
% Inbuts [ dr ]
% _3DOF_S = 3DOF spiral mode Approximation 
A_3DOF_S = [L_p_dash,L_r_dash,0 ;...  
          N_p_dash , N_r_dash , 0 ;... 
          1,0,0];
B_3DOF_S = [L_dr_dash;N_dr_dash;0];
C_3DOF_S=eye(3);
D_3DOF_S=zeros(3,1);
ss_3DOF_S = ss(A_3DOF_S,B_3DOF_S,C_3DOF_S,D_3DOF_S); 
Eigenvalues_3DOF_S= eig(A_3DOF_S);
%disp('Eigenvalues_3DOF_S:') 
%disp(Eigenvalues_3DOF_S)
TF_3DOF_S = tf(ss_3DOF_S);
% TF  with ruder
p_dr_3DOF_S = TF_3DOF_S(1,1);
r_dr_3DOF_S = TF_3DOF_S(2,1);
phi_dr_3DOF_S = TF_3DOF_S(3,1);

%% 3DOF Dutch Roll mode 
% States [beta ; p ; r]
% Inbuts [da ; dr]
% _3DOF_D = 3DOF Dutch Roll mode Approximation 
A_3DOF_D = [Yv,0,-1;...
             L_beta_dash,L_p_dash,0;...
             N_beta_dash,0,N_r_dash];
B_3DOF_D = [Y_star_da , Y_star_dr;... 
         L_da_dash , L_dr_dash;... 
         N_da_dash , N_dr_dash]; 
C_3DOF_D = eye(3);
D_3DOF_D=zeros(3,2);
ss_3DOF_D = ss(A_3DOF_D,B_3DOF_D,C_3DOF_D,D_3DOF_D); 
Eigenvalues_3DOF_D= eig(A_3DOF_D);
%disp('Eigenvalues_3DOF_D:') 
%disp(Eigenvalues_3DOF_D)
TF_3DOF_D = tf(ss_3DOF_D);
% TF  with alieron 
beta_da_3DOF_D = TF_3DOF_D(1,1);
p_da_3DOF_D = TF_3DOF_D(2,1);
r_da_3DOF_D = TF_3DOF_D(3,1);
% TF  with rudder
beta_dr_3DOF_D = TF_3DOF_D(1,2);
p_dr_3DOF_D = TF_3DOF_D(2,2);
r_dr_3DOF_D = TF_3DOF_D(3,2);
%%%%%
%% 2DOF Dutch Roll Approximation 
% States [beta ; r]   
% Inbuts [da ; dr]
% _2DOF = 2DOF Dutch Roll mode Approximation
A_2DOF = [Yv,-(1-Yr/u0);...
          N_beta_dash , N_r_dash];
B_2DOF = [Y_star_da , Y_star_dr;...
          N_da_dash , N_dr_dash];
C_2DOF = eye(2);
D_2DOF = zeros(2,2);
ss_2DOF = ss(A_2DOF,B_2DOF,C_2DOF,D_2DOF);
Eigenvalues_2DOF= eig(A_2DOF);
%disp('Eigenvalues_2DOF:') 
%disp(Eigenvalues_2DOF)
TF_2DOF = tf(ss_2DOF);
% TF  with alieron 
beta_da_2DOF = TF_2DOF(1,1);
r_da_2DOF = TF_2DOF(2,1);
% TF  with rudder
beta_dr_2DOF = TF_2DOF(1,2);
r_dr_2DOF = TF_2DOF(2,2);
%%%%%%%%%%%%%%
%% 1DOF Roll Approximation
% states [ p ]
% input [ da ]
% _1DOF = 1DOF Roll mode Approximatio
A_1DOF = L_p_dash;
B_1DOF = L_da_dash ;
C_1DOF = 1;
D_1DOF = 0;
ss_1DOF = ss(A_1DOF,B_1DOF,C_1DOF,D_1DOF);
Eigenvalues_1DOF= eig(A_1DOF);
%disp('Eigenvalues_1DOF:') 
%disp(Eigenvalues_1DOF)
TF_1DOF = tf(ss_1DOF);
p_da_1DOF = TF_1DOF(1,1);

%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Step response
% change in the aileron deflection 
aileron_inputs = deg2rad([1, 5, 10 ,25]); 
T_final = 100;
t = linspace(0, T_final, 1000);
filename = 'figures\Lateral Results\delta_a';

for j = 1:length(aileron_inputs) 
    aileron_input = aileron_inputs(j);
    SS_Lat_aileron_input = lat_ss(:,1); 

    [y_full, t] = step(SS_Lat_aileron_input * aileron_input, t);
    [y_3DOF_D, t] = step(ss_3DOF_D(:,1) * aileron_input, t);
    [y_2DOF, t] = step(ss_2DOF(:,1) * aileron_input, t);
    [y_1DOF, t] = step(ss_1DOF(:,1) * aileron_input, t);

    % Figure 1:  (beta, r)
    figure('units','normalized','outerposition',[0 0 1 1])
    % beta
    subplot(2,1,1);
    hold on ;
    plot(t, rad2deg(y_full(:,1)), 'b', 'LineWidth', 1.5); 
    plot(t, rad2deg(y_3DOF_D(:,1)), 'r', 'LineWidth', 1.5); 
    plot(t, rad2deg(y_2DOF(:,1)), '--g', 'LineWidth', 1.5); 
    ylabel('\beta (deg)'); grid on;
    legend('Full Response', '3DOF Dutch Roll App','2DOF Dutch Roll App');
    % r 
    subplot(2,1,2);
    hold on;
    plot(t, rad2deg(y_full(:,3)), 'b', 'LineWidth', 1.5); 
    plot(t, rad2deg(y_3DOF_D(:,3)), 'r', 'LineWidth', 1.5); 
    plot(t, rad2deg(y_2DOF(:,end)), '--g', 'LineWidth', 1.5); 
    ylabel('r (deg/s)'); grid on;
    legend('Full Response', '3DOF Dutch Roll App','2DOF Roll App');
    sgtitle([' aileron deflection - \delta_a = ', num2str(rad2deg(aileron_input)), '°']);
    set(findall(gcf,'type','line'),'linewidth',1.7);grid on ;legend ;
    saveas(gcf,fullfile(filename,strcat('BetaAndR_due_d_a = ',num2str(rad2deg(aileron_input)),'.png')));

    % Figure 2: (p, ϕ)
    figure('units','normalized','outerposition',[0 0 1 1])
    % p 
    subplot(2,1,1);
    hold on;
    plot(t, rad2deg(y_full(:,2)), 'b', 'LineWidth', 1.5); 
    plot(t, rad2deg(y_3DOF_D(:,2)), 'r', 'LineWidth', 1.5); 
    plot(t, rad2deg(y_1DOF(:,1)), 'g', 'LineWidth', 1.5); 
    ylabel('p (deg/s)'); grid on;
    legend('Full Response', '3DOF Dutch Roll App','1DOF Roll App');
    % ϕ 
    subplot(2,1,2);
    plot(t, rad2deg(y_full(:,4)), 'b', 'LineWidth', 1.5);
    ylabel('\phi (deg)'); xlabel('Time (seconds)'); grid on;
    legend('Full Response');
    sgtitle(['aileron deflection - \delta_a = ', num2str(rad2deg(aileron_input)), '°']);
    set(findall(gcf,'type','line'),'linewidth',1.7);grid on ;legend ;
    saveas(gcf,fullfile(filename,strcat('PAndPhi_due_d_a = ',num2str(rad2deg(aileron_input)),'.png')));
end

% change in the rudder deflection 
rudder_inputs = deg2rad([1, 5, 10, 25]); 
T_final = 100;
t = linspace(0, T_final, 1000);

% filename = 'figures\Lateral Results\delta_r';
% for j = 1:length(rudder_inputs) 
%     rudder_input = rudder_inputs(j);
%     SS_Lat_rudder_input = lat_ss(:,2); 
% 
%     [y_full, t] = step(SS_Lat_rudder_input * rudder_input, t);
%     [y_3DOF_S, t] = step(ss_3DOF_S(:,1) * rudder_input, t);
%     [y_3DOF_D, t] = step(ss_3DOF_D(:,2) * rudder_input, t);
%     [y_2DOF, t] = step(ss_2DOF(:,2) * rudder_input, t);
%     % Figure 1: (beta, r)
%     figure('units','normalized','outerposition',[0 0 1 1])
%     % beta 
%     subplot(2,1,1);
%     hold on ;
%     plot(t, rad2deg(y_full(:,1)), 'b', 'LineWidth', 1.5); 
%     plot(t, rad2deg(y_3DOF_D(:,1)), 'r', 'LineWidth', 1.5); 
%     plot(t, rad2deg(y_2DOF(:,1)), '--g', 'LineWidth', 1.5); 
%     ylabel('\beta (deg)'); grid on;
%     legend('Full Response', '3DOF Dutch Roll App','2DOF Dutch Roll App');
%     % r 
%     subplot(2,1,2);
%     hold on;
%     plot(t, rad2deg(y_full(:,3)), 'b', 'LineWidth', 1.5);
%     plot(t, rad2deg(y_3DOF_S(:,2)), 'k', 'LineWidth', 1.5); 
%     plot(t, rad2deg(y_3DOF_D(:,3)), 'r', 'LineWidth', 1.5); 
%     plot(t, rad2deg(y_2DOF(:,2)), '--g', 'LineWidth', 1.5); 
%     ylabel('r (deg/s)'); grid on;
%     legend('Full Response', '3DOF Spiral App','3DOF Dutch Roll App','2DOF Roll App');
%     sgtitle([' Rudder deflection - \delta_r = ', num2str(rad2deg(rudder_input)), '°']);
%     set(findall(gcf,'type','line'),'linewidth',1.7);grid on ;legend ;
%     saveas(gcf,fullfile(filename,strcat('BetaAndR_due_d_r = ',num2str(rad2deg(rudder_input)),'.png')));
% 
%     % Figure 2: (p, ϕ)
%     figure('units','normalized','outerposition',[0 0 1 1])
%     % p 
%    subplot(2,1,1);
%     hold on;
%     plot(t, rad2deg(y_full(:,2)), 'b', 'LineWidth', 1.5); 
%     plot(t, rad2deg(y_3DOF_S(:,1)), 'k-.', 'LineWidth', 1.5); 
%     plot(t, rad2deg(y_3DOF_D(:,2)), '--r', 'LineWidth', 1.5); 
%     plot(t, rad2deg(y_1DOF(:,1)), 'g', 'LineWidth', 1.5); 
%     ylabel('p (deg/s)'); grid on;
%     legend('Full Response','3DOF Spiral App', '3DOF Dutch Roll App','1DOF Roll App');
%     % ϕ 
%     subplot(2,1,2);
%     hold on ;
%     plot(t, rad2deg(y_full(:,4)), 'b', 'LineWidth', 1.5);
%      plot(t, rad2deg(y_3DOF_S(:,1)), 'k', 'LineWidth', 1.5); 
%     ylabel('\phi (deg)'); xlabel('Time (seconds)'); grid on;
%     legend('Full Response','3DOF Spiral App');
%     sgtitle(['Rudder deflection - \delta_r = ', num2str(rad2deg(rudder_input)), '°']);
%     set(findall(gcf,'type','line'),'linewidth',1.7);grid on ;legend ;
%     saveas(gcf,fullfile(filename,strcat('PAndPhi_due_d_r = ',num2str(rad2deg(rudder_input)),'.png')));
% end


%% Root locus for Lateral Dynamics 

RL_filename = 'figures\Lateral Results\Root Locus';
variable_names = {'\beta', 'p', 'r', '\phi', '\psi'};
% Define the modes
full_model_groups = {beta_da, p_da, r_da, phi_da, psi_da, beta_dr, p_dr, r_dr, phi_dr, psi_dr};
spiral_groups = {[], p_dr_3DOF_S, r_dr_3DOF_S, phi_dr_3DOF_S, []}; % Spiral mode for (p, r, phi)
dutch_3DOF_groups = {beta_da_3DOF_D, p_da_3DOF_D, r_da_3DOF_D, [], [], ...
                      beta_dr_3DOF_D, p_dr_3DOF_D, r_dr_3DOF_D, [], []}; 
dutch_2DOF_groups = {beta_da_2DOF, [], r_da_2DOF, [], [], ...
                      beta_dr_2DOF, [], r_dr_2DOF, [], []}; 
roll_1DOF_group = {[], p_da_1DOF, [], [], []}; % Roll mode affects only p

% Loop for (beta, p, r, phi, psi)
for i = 1:5
    figure( 'units','normalized','outerposition',[0 0 1 1])
    
    % psi
    if i == 5  
        subplot(1,2,1)
        rlocus(full_model_groups{i});
        sgtitle(['Root Locus Plot For (', variable_names{i},'\delta_{a}' ') Full Model'], 'Interpreter', 'none');
        set(findall(gcf,'type','line'),'linewidth',1.5);grid on ;
        % legend('$beta/\delta_{a} Full model$','interpreter','latex')

        subplot(1,2,2)
        rlocus(full_model_groups{i+5});
        sgtitle(['Root Locus Plot For (', variable_names{i},'\delta_{r}' ') Full Model'], 'Interpreter', 'none');
        set(findall(gcf,'type','line'),'linewidth',1.5);grid on ;
        % legend('$beta/\delta_{r} Full model$','interpreter','latex')
        
        saveas(gcf,fullfile(RL_filename,strcat(variable_names{i},  '.png')));

        
        continue;
    end
    % phi
    if i == 4  
        subplot(1,3,1);
        rlocus(full_model_groups{i});
        title(['(', variable_names{i}, ') Full Model'], 'Interpreter', 'none');
        set(findall(gcf,'type','line'),'linewidth',1.5);grid on ;

        subplot(1,2,3)
        rlocus(full_model_groups{i+5});
        sgtitle(['Root Locus Plot For (', variable_names{i},'\delta_{r}' ') Full Model'], 'Interpreter', 'none');
        set(findall(gcf,'type','line'),'linewidth',1.5);grid on ;

        subplot(1,3,3);
        rlocus(spiral_groups{i});
        title(['(', variable_names{i}, ') Spiral Mode'], 'Interpreter', 'none');
        set(findall(gcf,'type','line'),'linewidth',1.5);grid on ;

        saveas(gcf,fullfile(RL_filename,strcat(variable_names{i},  '.png')));
        continue;
    end
    
% For other cases: Use a 2×2 layout if there is more than one mode
    subplot(2,2,1);
    rlocus(full_model_groups{i});
    title(['(', variable_names{i}, ') Full Model'], 'Interpreter', 'none');

    if i <= length(spiral_groups) && ~isempty(spiral_groups{i})
        subplot(2,2,2);
        rlocus(spiral_groups{i});
        title(['(', variable_names{i}, ') Spiral Mode'], 'Interpreter', 'none');
        set(findall(gcf,'type','line'),'linewidth',1.5);grid on ;

        saveas(gcf,fullfile(RL_filename,strcat(variable_names{i},'.png')));
    end

    if i <= length(dutch_3DOF_groups) && ~isempty(dutch_3DOF_groups{i})
        subplot(2,2,3);
        rlocus(dutch_3DOF_groups{i});
        title(['(', variable_names{i}, ') Dutch Roll 3DOF'], 'Interpreter', 'none');
        set(findall(gcf,'type','line'),'linewidth',1.5);grid on ;

        saveas(gcf,fullfile(RL_filename,strcat(variable_names{i},'.png')));
    end

    if i <= length(dutch_2DOF_groups) && ~isempty(dutch_2DOF_groups{i})
        subplot(2,2,4);
        rlocus(dutch_2DOF_groups{i});
        title(['(', variable_names{i}, ') Dutch Roll 2DOF'], 'Interpreter', 'none');
        set(findall(gcf,'type','line'),'linewidth',1.5);grid on ;

        saveas(gcf,fullfile(RL_filename,strcat(variable_names{i},'.png')));
    end

    % 1DOF Roll Mode (if applicable)
    if i == 2 && ~isempty(roll_1DOF_group{i})
        figure( 'units','normalized','outerposition',[0 0 1 1])
        rlocus(roll_1DOF_group{i});
        title(['(', variable_names{i}, ') Roll Mode (1DOF)'], 'Interpreter', 'none');
        set(findall(gcf,'type','line'),'linewidth',1.5);grid on ;

        saveas(gcf,fullfile(RL_filename,strcat(variable_names{i},'.png')));
    end
end
% %% Bode Plot for Lateral Dynamics 
% variable_names = {'\beta', 'p', 'r', '\phi', '\psi'};
% 
% % Define the modes
% full_model_groups = {beta_da, p_da, r_da, phi_da, psi_da, beta_dr, p_dr, r_dr, phi_dr, psi_da};
% spiral_groups = {[], p_dr_3DOF_S, r_dr_3DOF_S, phi_dr_3DOF_S, []}; % Spiral mode for (p, r, phi)
% dutch_3DOF_groups = {beta_da_3DOF_D, p_da_3DOF_D, r_da_3DOF_D, [], [], ...
%                       beta_dr_3DOF_D, p_dr_3DOF_D, r_dr_3DOF_D, [], []}; 
% dutch_2DOF_groups = {beta_da_2DOF, [], r_da_2DOF, [], [], ...
%                       beta_dr_2DOF, [], r_dr_2DOF, [], []}; 
% roll_1DOF_group = {[], p_da_1DOF, [], [], []}; % Roll mode affects only p
% 
% % Loop for (beta, p, r, phi, psi)
% for i = 1:5
%     figure('units','normalized','outerposition',[0 0 1 1])
% 
%     % psi
%     if i == 5  
%         bode(full_model_groups{i});
%         title(['(', variable_names{i}, ') Full Model'], 'Interpreter', 'none');
%         continue;
%     end
%     % phi
%     if i == 4  
%         subplot(1,2,1);
%         bode(full_model_groups{i});
%         title(['(', variable_names{i}, ') Full Model'], 'Interpreter', 'none');
% 
%         subplot(1,2,2);
%         bode(spiral_groups{i});
%         title(['(', variable_names{i}, ') Spiral Mode'], 'Interpreter', 'none');
% 
%         continue;
%     end
% 
%     % For other cases: Use a 2×2 layout if there is more than one mode
%     subplot(2,2,1);
%     bode(full_model_groups{i});
%     title(['(', variable_names{i}, ') Full Model'], 'Interpreter', 'none');
% 
%     if i <= length(spiral_groups) && ~isempty(spiral_groups{i})
%         subplot(2,2,2);
%         bode(spiral_groups{i});
%         title(['(', variable_names{i}, ') Spiral Mode'], 'Interpreter', 'none');
%     end
% 
%     if i <= length(dutch_3DOF_groups) && ~isempty(dutch_3DOF_groups{i})
%         subplot(2,2,3);
%         bode(dutch_3DOF_groups{i});
%         title(['(', variable_names{i}, ') Dutch Roll 3DOF'], 'Interpreter', 'none');
%     end
% 
%     if i <= length(dutch_2DOF_groups) && ~isempty(dutch_2DOF_groups{i})
%         subplot(2,2,4);
%         bode(dutch_2DOF_groups{i});
%         title(['(', variable_names{i}, ') Dutch Roll 2DOF'], 'Interpreter', 'none');
%     end
% 
%     % 1DOF Roll Mode (if applicable)
%     if i == 2 && ~isempty(roll_1DOF_group{i})
%         figure('units','normalized','outerposition',[0 0 1 1])
%         bode(roll_1DOF_group{i});
%         title(['(', variable_names{i}, ') Roll Mode (1DOF)'], 'Interpreter', 'none');
%     end
% end