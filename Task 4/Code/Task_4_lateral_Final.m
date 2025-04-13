clc 
clear vars 
close all 
%%%%%%%%%%%%%%%
%% get data 
run("Calculations.m");
% tempvariatdash = num2cell(SD_lat_dash);
% [Yv, Yb, Lbd, Nbd, Lpd, Npd, Lrd, Nrd, Yda, Ydr, Ldad, Ndad, Ldrd, Ndrd] = deal(tempvariatdash{:});
% Lvd = Lbd / Vtotal_0;
% Nvd = Nbd/Vtotal_0;
wdot = zeros(1,n+1);
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
A_lat = [Yb/Vtotal_0 , (w0+Yp)/Vtotal_0 , (-u0+Yr)/Vtotal_0 , g*cos(theta0)/Vtotal_0 , 0;... 
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
disp('Eigenvalues:') 
disp(Eigenvalues)
[e_vecs_lat,e_vals_lat]= eig(A_lat);
lat_TF = tf(lat_ss);
% TF  with aliron
beta_da = lat_TF(1,1)+beta0;
p_da = lat_TF(2,1)+p0;
r_da = lat_TF(3,1)+r0;
phi_da = lat_TF(4,1)+phi0;
psi_da = lat_TF(5,1)+psi0;
% TF  with rudder
beta_dr = lat_TF(1,2)+beta0;
p_dr = lat_TF(2,2)+p0;
r_dr = lat_TF(3,2)+r0;
phi_dr = lat_TF(4,2)+phi0;
psi_dr = lat_TF(5,2)+psi0;
%% Modes Approximation
%% 3DOF mode spiral
% States [p ; r ; phi]
% Inbuts [ dr ]
% _3DOF_S = 3DOF spiral mode Approximation 
A_3DOF_S = [L_p_dash,L_r_dash,0 ;...  
          N_p_dash , N_r_dash , 0 ;... 
          1,tan(theta0),0];
B_3DOF_S = [L_da_dash , L_dr_dash;...
             N_da_dash , N_dr_dash;...
             0 , 0];
C_3DOF_S=eye(3);
D_3DOF_S=zeros(size(B_3DOF_S));
ss_3DOF_S = ss(A_3DOF_S,B_3DOF_S,C_3DOF_S,D_3DOF_S); 
Eigenvalues_3DOF_S= eig(A_3DOF_S);
disp('Eigenvalues_3DOF_S:') 
disp(Eigenvalues_3DOF_S)
TF_3DOF_S = tf(ss_3DOF_S);
% TF  with aliron
p_da_3DOF_S = TF_3DOF_S(1,1)+p0;
r_da_3DOF_S = TF_3DOF_S(2,1)+r0;
phi_da_3DOF_S = TF_3DOF_S(3,1)+phi0;
% TF  with ruder
p_dr_3DOF_S = TF_3DOF_S(1,2)+p0;
r_dr_3DOF_S = TF_3DOF_S(2,2)+r0;
phi_dr_3DOF_S = TF_3DOF_S(3,2)+phi0;

%% 3DOF Dutch Roll mode 
% States [beta ; p ; r]
% Inbuts [da ; dr]
% _3DOF_D = 3DOF Dutch Roll mode Approximation 
A_3DOF_D = [Yv,(Yp+w0)/Vtotal_0,(Yr-u0)/Vtotal_0;...
             L_beta_dash,L_p_dash,0;...
             N_beta_dash,0,N_r_dash];
B_3DOF_D = [Y_star_da , Y_star_dr;... 
         L_da_dash , L_dr_dash;... 
         N_da_dash , N_dr_dash]; 
C_3DOF_D = eye(3);
D_3DOF_D=zeros(size(B_3DOF_D));
ss_3DOF_D = ss(A_3DOF_D,B_3DOF_D,C_3DOF_D,D_3DOF_D); 
Eigenvalues_3DOF_D= eig(A_3DOF_D);
disp('Eigenvalues_3DOF_D:') 
disp(Eigenvalues_3DOF_D)
TF_3DOF_D = tf(ss_3DOF_D);
% TF  with alieron 
beta_da_3DOF_D = TF_3DOF_D(1,1)+beta0;
p_da_3DOF_D = TF_3DOF_D(2,1)+p0;
r_da_3DOF_D = TF_3DOF_D(3,1)+r0;
% TF  with rudder
beta_dr_3DOF_D = TF_3DOF_D(1,2)+beta0;
p_dr_3DOF_D = TF_3DOF_D(2,2)+p0;
r_dr_3DOF_D = TF_3DOF_D(3,2)+r0;
%%%%%
%% 2DOF Dutch Roll Approximation 
% States [beta ; r]   
% Inbuts [da ; dr]
% _2DOF = 2DOF Dutch Roll mode Approximation
A_2DOF = [Yv,(Yr-u0)/Vtotal_0-tan(theta0)*(Yp+w0)/Vtotal_0;...
          N_beta_dash , N_r_dash-tan(theta0)*N_p_dash];
B_2DOF = [Y_star_da , Y_star_dr;...
          N_da_dash , N_dr_dash];
C_2DOF = eye(2);
D_2DOF = zeros(size(B_2DOF));
ss_2DOF = ss(A_2DOF,B_2DOF,C_2DOF,D_2DOF);
Eigenvalues_2DOF= eig(A_2DOF);
disp('Eigenvalues_2DOF:') 
disp(Eigenvalues_2DOF)
TF_2DOF = tf(ss_2DOF);
% TF  with alieron 
beta_da_2DOF = TF_2DOF(1,1)+beta0;
r_da_2DOF = TF_2DOF(2,1)+r0;
% TF  with rudder
beta_dr_2DOF = TF_2DOF(1,2)+beta0;
r_dr_2DOF = TF_2DOF(2,2)+r0;
%%%%%%%%%%%%%%
%% 1DOF Roll Approximation
% states [ p ]
% input [ da ]
% _1DOF = 1DOF Roll mode Approximatio
A_1DOF = L_p_dash;
B_1DOF = L_da_dash ;
C_1DOF = eye(1); 
D_1DOF = zeros(size(B_1DOF));
ss_1DOF = ss(A_1DOF,B_1DOF,C_1DOF,D_1DOF);
Eigenvalues_1DOF= eig(A_1DOF);
disp('Eigenvalues_1DOF:') 
disp(Eigenvalues_1DOF)
TF_1DOF = tf(ss_1DOF);
p_da_1DOF = TF_1DOF(1,1)+p0;

%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Step response
% change in the aileron deflection 
aileron_inputs = deg2rad([1, 5, 10 ,25]); 
tfinal = 100;
T_final = 100;
t = linspace(0, T_final, 1000);
filename = 'figures\Lateral Results\response\delta_a';

delta_aileron = 0;
delta_rudder = 0;
delta_elevator =0;
delta_thrust = 0;

for j = 1:length(aileron_inputs) 
    aileron_input = aileron_inputs(j);
    SS_Lat_aileron_input = lat_ss(:,1); 

    delta_aileron = aileron_input;
    simOut = sim('Simulator21.slx');

    simData = {simOut.u_simulink, simOut.v_simulink, simOut.w_simulink, ...
                   simOut.p_simulink, simOut.q_simulink, simOut.r_simulink, ...
                   simOut.phi_simulink, simOut.theta_simulink, simOut.psi_simulink, ...
                   simOut.x_simulink, simOut.y_simulink, simOut.z_simulink, ...
                   simOut.alpha_simulink, simOut.beta_simulink};

    [y_full, t] = step(SS_Lat_aileron_input * aileron_input, t);
    [y_3DOF_S, t] = step(ss_3DOF_S(:,1) * aileron_input, t);
    [y_3DOF_D, t] = step(ss_3DOF_D(:,1) * aileron_input, t);
    [y_2DOF, t] = step(ss_2DOF(:,1) * aileron_input, t);
    [y_1DOF, t] = step(ss_1DOF(:,1) * aileron_input, t);

    % Figure 1:  (beta, r)
    figure('units','normalized','outerposition',[0 0 1 1])
    % beta
    subplot(2,1,1);
    hold on ;
    plot(t, rad2deg(y_full(:,1))+rad2deg(beta0), 'b', 'LineWidth', 1.5);
    plot(t, rad2deg(y_3DOF_D(:,1))+rad2deg(beta0), 'r', 'LineWidth', 1.5); 
    plot(t, rad2deg(y_2DOF(:,1))+rad2deg(beta0), '--g', 'LineWidth', 1.5);
    plot(simData{14}.time, simData{14}.data, '--m', 'LineWidth', 1.5);
    ylabel('\beta (deg)'); grid on;
    legend('Full model','3DOF Dutch Roll App','2DOF Dutch Roll App', 'Non Linear');
    % r 
    subplot(2,1,2);
    hold on;
    plot(t, (y_full(:,3))+(r0), 'b', 'LineWidth', 1.5); 
    plot(t, (y_3DOF_S(:,2))+(r0), 'k', 'LineWidth', 1.5); 
    plot(t, (y_3DOF_D(:,3))+(r0), 'r', 'LineWidth', 1.5); 
    plot(t, (y_2DOF(:,end))+(r0), '--g', 'LineWidth', 1.5);
    plot(simData{6}.time, (simData{6}.data), '--m', 'LineWidth', 1.5);
    ylabel('r (rad/s)'); grid on;
    legend('Full model','3DOF Spiral App', '3DOF Dutch Roll App','2DOF Roll App', 'Non Linear');
    sgtitle([' aileron deflection - \delta_a = ', num2str(rad2deg(aileron_input)), '°']);
    set(findall(gcf,'type','line'),'linewidth',1.7);grid on ;legend ;
    saveas(gcf,fullfile(filename,strcat('BetaAndR_due_d_a = ',num2str(rad2deg(aileron_input)),'.png')));

    % Figure 2: (p, ϕ)
    figure('units','normalized','outerposition',[0 0 1 1])
    % p 
    subplot(2,1,1);
    hold on;
    plot(t, (y_full(:,2))+(p0), 'b', 'LineWidth', 1.5); 
    plot(t, (y_3DOF_S(:,1))+(p0), 'k', 'LineWidth', 1.5); 
    plot(t, (y_3DOF_D(:,2))+(p0), 'r', 'LineWidth', 1.5); 
    plot(t, (y_1DOF(:,1))+(p0), 'g', 'LineWidth', 1.5);
    plot(simData{4}.time, (simData{4}.data), '--m', 'LineWidth', 1.5);
    ylabel('p (rad/s)'); grid on;
    legend('Full model','3DOF Spiral App', '3DOF Dutch Roll App','1DOF Roll App', 'Non Linear');
    % ϕ 
    subplot(2,1,2);
    hold on ;
    plot(t, rad2deg(y_full(:,4))+rad2deg(phi0), 'b', 'LineWidth', 1.5);
    plot(t, rad2deg(y_3DOF_S(:,3))+rad2deg(phi0), 'k', 'LineWidth', 1.5); 
    plot(simData{7}.time, simData{7}.data, '--m', 'LineWidth', 1.5);
    ylabel('\phi (deg)'); xlabel('Time (seconds)'); grid on;
    legend('Full model', '3DOF Spiral App','Non Linear');
    sgtitle(['aileron deflection - \delta_a = ', num2str(rad2deg(aileron_input)), '°']);
    set(findall(gcf,'type','line'),'linewidth',1.7);grid on ;legend ;
    saveas(gcf,fullfile(filename,strcat('PAndPhi_due_d_a = ',num2str(rad2deg(aileron_input)),'.png')));
end

%% change in the rudder deflection 
rudder_inputs = deg2rad([1, 5, 10, 25]); 
T_final = 100;
t = linspace(0, T_final, 1000);
filename = 'figures\Lateral Results\response\delta_r';
delta_aileron = 0;
delta_rudder = 0;
delta_elevator =0;
delta_thrust = 0;
for j = 1:length(rudder_inputs) 
    rudder_input = rudder_inputs(j);
    SS_Lat_rudder_input = lat_ss(:,2);

    delta_rudder = rudder_input;
    simOut = sim('Simulator21.slx');
    simData = {simOut.u_simulink, simOut.v_simulink, simOut.w_simulink, ...
                   simOut.p_simulink, simOut.q_simulink, simOut.r_simulink, ...
                   simOut.phi_simulink, simOut.theta_simulink, simOut.psi_simulink, ...
                   simOut.x_simulink, simOut.y_simulink, simOut.z_simulink, ...
                   simOut.alpha_simulink, simOut.beta_simulink};

    [y_full, t] = step(SS_Lat_rudder_input * rudder_input, t);
    [y_3DOF_S, t] = step(ss_3DOF_S(:,2) * rudder_input, t);
    [y_3DOF_D, t] = step(ss_3DOF_D(:,2) * rudder_input, t);
    [y_2DOF, t] = step(ss_2DOF(:,2) * rudder_input, t);
    % Figure 1: (beta, r)
    figure('units','normalized','outerposition',[0 0 1 1])
    % beta 
    subplot(2,1,1);
    hold on ;
    plot(t, rad2deg(y_full(:,1))+rad2deg(beta0), 'b', 'LineWidth', 1.5); 
    plot(t, rad2deg(y_3DOF_D(:,1))+rad2deg(beta0), 'r', 'LineWidth', 1.5); 
    plot(t, rad2deg(y_2DOF(:,1))+rad2deg(beta0), '--g', 'LineWidth', 1.5);
    plot(simData{14}.time, simData{14}.data, '--m', 'LineWidth', 1.5);
    ylabel('\beta (deg)'); grid on;
    legend('Full model', '3DOF Dutch Roll App','2DOF Dutch Roll App', 'Non Linear');
    % r 
    subplot(2,1,2);
    hold on;
    plot(t, (y_full(:,3))+(r0), 'b', 'LineWidth', 1.5);
    plot(t, (y_3DOF_S(:,2))+(r0), 'k', 'LineWidth', 1.5); 
    plot(t, (y_3DOF_D(:,3))+(r0), 'r', 'LineWidth', 1.5); 
    plot(t, (y_2DOF(:,2))+(r0), '--g', 'LineWidth', 1.5);
    plot(simData{6}.time, (simData{6}.data), '--m', 'LineWidth', 1.5);
    ylabel('r (rad/s)'); grid on;
    legend('Full model', '3DOF Spiral App','3DOF Dutch Roll App','2DOF Roll App', 'Non Linear');
    sgtitle([' Rudder deflection - \delta_r = ', num2str(rad2deg(rudder_input)), '°']);
    set(findall(gcf,'type','line'),'linewidth',1.7);grid on ;legend ;
    saveas(gcf,fullfile(filename,strcat('BetaAndR_due_d_r = ',num2str(rad2deg(rudder_input)),'.png')));

   % Figure 2: (p, ϕ)
    figure('units','normalized','outerposition',[0 0 1 1])
    % p 
   subplot(2,1,1);
    hold on;
    plot(t, (y_full(:,2))+(p0), 'b', 'LineWidth', 1.5); 
    plot(t, (y_3DOF_S(:,1))+(p0), 'k-.', 'LineWidth', 1.5); 
    plot(t, (y_3DOF_D(:,2))+(p0), '--r', 'LineWidth', 1.5); 
    plot(simData{4}.time, (simData{4}.data), '--m', 'LineWidth', 1.5);
    %plot(t, (y_1DOF(:,1))+(p0), 'g', 'LineWidth', 1.5);
    ylabel('p (rad/s)'); grid on;
    legend('Full model','3DOF Spiral App', '3DOF Dutch Roll App','Non Linear','1DOF Roll App');
    % ϕ 
    subplot(2,1,2);
    hold on ;
    plot(t, rad2deg(y_full(:,4))+rad2deg(phi0), 'b', 'LineWidth', 1.5);
    plot(t, rad2deg(y_3DOF_S(:,3))+rad2deg(phi0), 'k', 'LineWidth', 1.5);
    plot(simData{7}.time, simData{7}.data, '--m', 'LineWidth', 1.5);
    ylabel('\phi (deg)'); xlabel('Time (seconds)'); grid on;
    legend('Full model','3DOF Spiral App', 'Non Linear');
    sgtitle(['Rudder deflection - \delta_r = ', num2str(rad2deg(rudder_input)), '°']);
    set(findall(gcf,'type','line'),'linewidth',1.7);grid on ;legend ;
    saveas(gcf,fullfile(filename,strcat('PAndPhi_due_d_r = ',num2str(rad2deg(rudder_input)),'.png')));
end
% 
%%
% Root locus for Lateral Dynamics (Aileron Only)
RL_a_filename = 'figures\Lateral Results\Root Locus\delta_a';

variable_names = {'\beta', 'p', 'r', '\phi', '\psi'};
full_model_groups = {beta_da, p_da, r_da, phi_da, psi_da}; 
dutch_3DOF_groups = {beta_da_3DOF_D, p_da_3DOF_D, r_da_3DOF_D, [], []}; 
dutch_2DOF_groups = {beta_da_2DOF, [], r_da_2DOF, [], []}; 
roll_1DOF_group = {[], p_da_1DOF, [], [], []}; 
spiral_3DOF_groups = {[], p_da_3DOF_S, r_da_3DOF_S, phi_da_3DOF_S, []};

for i = 1:5
    figure('units','normalized','outerposition',[0 0 1 1])
    folderPath = fullfile(RL_a_filename, variable_names{i});
    if ~exist(folderPath, 'dir')
        mkdir(folderPath);
    end

    % Special case: ψ (psi)
    if i == 5  
        rlocus(full_model_groups{i});
        title(['(', variable_names{i}, ') Full Model (Aileron)'], 'Interpreter', 'none');
        set(findall(gcf,'type','line'),'linewidth',1.5); grid on;
        saveas(gcf, fullfile(RL_a_filename, strcat(variable_names{i}, '/\delta_{a} ','.png')));
        continue;
    end

    % Special case: φ (phi) --> Full + Spiral (using subplot instead of tiledlayout)
    if i == 4
        subplot(1,2,1)
        rlocus(full_model_groups{i});
        title(['(', variable_names{i}, ') Full Model (Aileron)'], 'Interpreter', 'none');
        set(findall(gcf,'type','line'),'linewidth',1.5); grid on;
        xlabel('Real Axis (seconds^{-1})')
        ylabel('Imaginary Axis (seconds^{-1})')

        subplot(1,2,2)
        rlocus(spiral_3DOF_groups{i});
        title(['(', variable_names{i}, ') Spiral Mode (Aileron)'], 'Interpreter', 'none');
        set(findall(gcf,'type','line'),'linewidth',1.5); grid on;
        xlabel('Real Axis (seconds^{-1})')
        ylabel('Imaginary Axis (seconds^{-1})')

        saveas(gcf, fullfile(RL_a_filename, strcat(variable_names{i}, '/\delta_{a} ','.png')));
        continue;
    end

   %%%
    subplot_idx = 1;

    % Full model
    subplot(2,2,subplot_idx);
    rlocus(full_model_groups{i});
    title(['(', variable_names{i}, ') Full Model (Aileron)'], 'Interpreter', 'none');
    set(findall(gcf,'type','line'),'linewidth',1.5); grid on;
    subplot_idx = subplot_idx + 1;

    % Dutch 3DOF
    if i <= length(dutch_3DOF_groups) && ~isempty(dutch_3DOF_groups{i})
        subplot(2,2,subplot_idx);
        rlocus(dutch_3DOF_groups{i});
        title(['(', variable_names{i}, ') Dutch Roll 3DOF (Aileron)'], 'Interpreter', 'none');
        set(findall(gcf,'type','line'),'linewidth',1.5); grid on;
        subplot_idx = subplot_idx + 1;
    end

    % Dutch 2DOF
    if i <= length(dutch_2DOF_groups) && ~isempty(dutch_2DOF_groups{i})
        subplot(2,2,subplot_idx);
        rlocus(dutch_2DOF_groups{i});
        title(['(', variable_names{i}, ') Dutch Roll 2DOF (Aileron)'], 'Interpreter', 'none');
        set(findall(gcf,'type','line'),'linewidth',1.5); grid on;
        subplot_idx = subplot_idx + 1;
    end

    % Spiral 3DOF
    if i <= length(spiral_3DOF_groups) && ~isempty(spiral_3DOF_groups{i})
        subplot(2,2,subplot_idx);
        rlocus(spiral_3DOF_groups{i});
        title(['(', variable_names{i}, ') Spiral Mode (Aileron)'], 'Interpreter', 'none');
        set(findall(gcf,'type','line'),'linewidth',1.5); grid on;
        subplot_idx = subplot_idx + 1;
    end

    % Roll 1DOF
    if i == 2 && ~isempty(roll_1DOF_group{i})
        subplot(2,2,subplot_idx);
        rlocus(roll_1DOF_group{i});
        title(['(', variable_names{i}, ') Roll Mode (Aileron)'], 'Interpreter', 'none');
        set(findall(gcf,'type','line'),'linewidth',1.5); grid on;
    end

    saveas(gcf, fullfile(RL_a_filename, strcat(variable_names{i}, '/\delta_{a} ','.png')));
end


% Root locus for Lateral Dynamics (Rudder Only)

RL_r_filename = 'figures\Lateral Results\Root Locus\delta_r';
variable_names = {'\beta', 'p', 'r', '\phi', '\psi'};
full_model_groups = {beta_dr, p_dr, r_dr, phi_dr, psi_dr}; 
spiral_3DOF_groups = {[], p_dr_3DOF_S, r_dr_3DOF_S, phi_dr_3DOF_S, []}; % Spiral mode موجود فقط لبعض المتغيرات
dutch_3DOF_groups = {beta_dr_3DOF_D, p_dr_3DOF_D, r_dr_3DOF_D, [], []}; 
dutch_2DOF_groups = {beta_dr_2DOF, [], r_dr_2DOF, [], []}; 

for i = 1:5
    figure( 'units','normalized','outerposition',[0 0 1 1])
    folderPath = fullfile(RL_r_filename, variable_names{i});
        if ~exist(folderPath, 'dir')
            mkdir(folderPath);
        end

    if i == 5  
        rlocus(full_model_groups{i});
        title(['(', variable_names{i}, ') Full Model (Rudder)'], 'Interpreter', 'none');
        set(findall(gcf,'type','line'),'linewidth',1.5);grid on ;
        saveas(gcf,fullfile(RL_r_filename,strcat(variable_names{i}, '/\delta_{r} ','.png')));

        continue;
    end

    if i == 4  
        subplot(1,2,1);
        rlocus(full_model_groups{i});
        title(['(', variable_names{i}, ') Full Model (Rudder)'], 'Interpreter', 'none');
        set(findall(gcf,'type','line'),'linewidth',1.5);grid on ;

        subplot(1,2,2);
        rlocus(spiral_3DOF_groups{i});
        title(['(', variable_names{i}, ') Spiral Mode (Rudder)'], 'Interpreter', 'none');
        set(findall(gcf,'type','line'),'linewidth',1.5);grid on ;
        saveas(gcf,fullfile(RL_r_filename,strcat(variable_names{i}, '/\delta_{r} ','.png')));

        continue;
    end

    %For other cases: Use a 2×2 layout
    subplot(2,2,1);
    rlocus(full_model_groups{i});
    title(['(', variable_names{i}, ') Full Model (Rudder)'], 'Interpreter', 'none');
    set(findall(gcf,'type','line'),'linewidth',1.5);grid on ;

    if i <= length(spiral_3DOF_groups) && ~isempty(spiral_3DOF_groups{i})
        subplot(2,2,2);
        rlocus(spiral_3DOF_groups{i});
        title(['(', variable_names{i}, ') Spiral Mode (Rudder)'], 'Interpreter', 'none');
        set(findall(gcf,'type','line'),'linewidth',1.5);grid on ;
        saveas(gcf,fullfile(RL_r_filename,strcat(variable_names{i}, '/\delta_{r} ','.png')));

    end

    if i <= length(dutch_3DOF_groups) && ~isempty(dutch_3DOF_groups{i})
        subplot(2,2,3);
        rlocus(dutch_3DOF_groups{i});
        title(['(', variable_names{i}, ') Dutch Roll 3DOF (Rudder)'], 'Interpreter', 'none');
        set(findall(gcf,'type','line'),'linewidth',1.5);grid on ;
        saveas(gcf,fullfile(RL_r_filename,strcat(variable_names{i}, '/\delta_{r} ','.png')));

    end

    if i <= length(dutch_2DOF_groups) && ~isempty(dutch_2DOF_groups{i})
        subplot(2,2,4);
        rlocus(dutch_2DOF_groups{i});
        title(['(', variable_names{i}, ') Dutch Roll 2DOF (Rudder)'], 'Interpreter', 'none');
        set(findall(gcf,'type','line'),'linewidth',1.5);grid on ;
        saveas(gcf,fullfile(RL_r_filename,strcat(variable_names{i}, '/\delta_{r} ','.png')));

    end
end
%% Bode Plot for Lateral Dynamics (Aileron Only)
%%% bode plot (delta_Aileron)
Bode_a_filename = 'figures\Lateral Results\Bode Plots\Aileron';
variable_names = {'\beta', 'p', 'r', '\phi', '\psi'};
full_model_groups = {beta_da, p_da, r_da, phi_da, psi_da}; 
dutch_3DOF_groups = {beta_da_3DOF_D, p_da_3DOF_D, r_da_3DOF_D, [], []}; 
dutch_2DOF_groups = {beta_da_2DOF, [], r_da_2DOF, [], []}; 
roll_1DOF_group = {[], p_da_1DOF, [], [], []}; 
spiral_3DOF_groups = {[], p_da_3DOF_S, r_da_3DOF_S, phi_da_3DOF_S, []}; % New Spiral 3DOF group

for i = 1:5
    figure('units','normalized','outerposition',[0 0 1 1])
    folderPath = fullfile(Bode_a_filename, variable_names{i});
    if ~exist(folderPath, 'dir')
        mkdir(folderPath);
    end

    labels = {}; % initialize legend labels

    bode(full_model_groups{i}, 'b'); hold on;
    labels{end+1} = 'Full Model';

    if i <= length(dutch_3DOF_groups) && ~isempty(dutch_3DOF_groups{i})
        bode(dutch_3DOF_groups{i}, 'r--'); hold on;
        labels{end+1} = 'Dutch Roll 3DOF';
    end
    if i <= length(dutch_2DOF_groups) && ~isempty(dutch_2DOF_groups{i})
        bode(dutch_2DOF_groups{i}, 'k-.'); hold on;
        labels{end+1} = 'Dutch Roll 2DOF';
    end
    if i == 2 && ~isempty(roll_1DOF_group{i})
        bode(roll_1DOF_group{i}, 'g:'); hold on;
        labels{end+1} = 'Roll Mode';
    end
    if i <= length(spiral_3DOF_groups) && ~isempty(spiral_3DOF_groups{i})
        bode(spiral_3DOF_groups{i}, 'm--'); hold on;
        labels{end+1} = 'Spiral Mode';
    end

    hold off;
    title(['(', variable_names{i}, ') Bode Plot Comparison (Aileron)'], 'Interpreter', 'none');
    legend(labels, 'Location', 'best');
    set(findall(gcf,'type','line'),'linewidth',1.5);
    grid on;

    saveas(gcf, fullfile(Bode_a_filename, strcat(variable_names{i}, '/\delta_{a} ','.png')));
end

%% bode plot (delta_rudder)
Bode_r_filename = 'figures\Lateral Results\Bode Plots\Rudder';
variable_names = {'\beta', 'p', 'r', '\phi', '\psi'};
full_model_groups = {beta_dr, p_dr, r_dr, phi_dr, psi_dr}; 
spiral_3DOF_groups = {[], p_dr_3DOF_S, r_dr_3DOF_S, phi_dr_3DOF_S, []}; 
dutch_3DOF_groups = {beta_dr_3DOF_D, p_dr_3DOF_D, r_dr_3DOF_D, [], []}; 
dutch_2DOF_groups = {beta_dr_2DOF, [], r_dr_2DOF, [], []}; 

for i = 1:5
    figure('units','normalized','outerposition',[0 0 1 1])
    folderPath = fullfile(Bode_r_filename, variable_names{i});
    if ~exist(folderPath, 'dir')
        mkdir(folderPath);
    end

    labels = {}; % initialize labels

    bode(full_model_groups{i}, 'b'); hold on;
    labels{end+1} = 'Full Model';

    if i <= length(spiral_3DOF_groups) && ~isempty(spiral_3DOF_groups{i})
        bode(spiral_3DOF_groups{i}, 'm--'); hold on;
        labels{end+1} = 'Spiral Mode';
    end
    if i <= length(dutch_3DOF_groups) && ~isempty(dutch_3DOF_groups{i})
        bode(dutch_3DOF_groups{i}, 'r-.'); hold on;
        labels{end+1} = 'Dutch Roll 3DOF';
    end
    if i <= length(dutch_2DOF_groups) && ~isempty(dutch_2DOF_groups{i})
        bode(dutch_2DOF_groups{i}, 'k:'); hold on;
        labels{end+1} = 'Dutch Roll 2DOF';
    end

    hold off;
    title(['(', variable_names{i}, ') Bode Plot Comparison (Rudder)'], 'Interpreter', 'none');
    legend(labels);
    set(findall(gcf,'type','line'),'linewidth',1.5);
    grid on;

    saveas(gcf, fullfile(Bode_r_filename, strcat(variable_names{i}, '/\delta_{r} ','.png')));
end
