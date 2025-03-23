clc; clearvars; close all;

run("Calculations.m");
% benchmark_data = load('Benchmark_B747_FC5.mat');
importfile('Benchmark_B747_FC5.mat')
% disp(fieldnames(benchmark_data)); % Display variable names inside the MAT file

% 
% delta_aileron   = [0;5;-5;0;0;0;0;0;0];
% delta_rudder    = [0;0;0;5;-5;0;0;0;0];
% delta_elevator  = [0;0;0;0;0;5;-5;0;0];
% delta_thrust    = [0;0;0;0;0;0;0;1000;10000];
% delta_control   = [deg2rad([delta_aileron, delta_rudder, delta_elevator]), delta_thrust];

%% Control action
delta_aileron = aircraft_data(57);
delta_rudder = aircraft_data(58);
delta_elevator = aircraft_data(59);
delta_thrust = aircraft_data(60);

delta_control = [deg2rad([delta_aileron, delta_rudder, delta_elevator]), delta_thrust];

% for i = 1:2
%     % delta_control = Input(i,:);
%         %% ========================== Runge-Kutta 4th Order (RK4) ==========================
%     for j = 2:n+1
%         [Forces(:,j),Moments(:,j)] = Airframe_Model(states_0,states_RK4(:,j-1),wdot(j-1),mass,Inertia,gravity,delta_control(i,:), Matrix_states, Matrix_controls);
%         [states_RK4(:,j), wdot(j)] = raunge_kutta_4(t_vec(j), Forces(:,j), Moments(:,j), states_RK4(:,j-1), mass, Inertia, I_inv, dt);    
%     end
%     %% Call validation_helper.m
%     % valadation_help(states_RK4, t_vec)    
%     %% simulink  
%     Simulator = sim('R21a.slx');
%     %% plot valadation
%     % plotValidation(t_vec, states_RK4, Simulator);
%     plot_Validation(t_vec, states_RK4, Simulator);
%     figHandles = findall(groot, 'Type', 'figure');
%     for k = 1:length(figHandles)
%         figure(figHandles(k)); % Bring figure to focus
%         set(figHandles(k), 'Units', 'normalized', 'OuterPosition', [0 0 1 1]); % Fullscreen mode
%         filename = sprintf('figure_%d.svg', k);
%         saveas(figHandles(k), filename);
%     end
% end

% for i = 1:9
%     % Run RK4 Simulation
%     for j = 2:n+1
%         [Forces(:,j), Moments(:,j)] = Airframe_Model(states_0, states_RK4(:,j-1), wdot(j-1), mass, Inertia, gravity, delta_control(i,:), Matrix_states, Matrix_controls);
%         [states_RK4(:,j), wdot(j)] = raunge_kutta_4(t_vec(j), Forces(:,j), Moments(:,j), states_RK4(:,j-1), mass, Inertia, I_inv, dt);
%     end
% 
%     % Run Simulink
%     Simulator = sim('R21a.slx');
% 
%     % Plot validation
%     plot_Validation(t_vec, states_RK4, Simulator , delta_control(i, :));
% end



% Run RK4 Simulation
for j = 2:n+1
    [Forces(:,j), Moments(:,j)] = Airframe_Model(states_0, states_RK4(:,j-1), wdot(j-1), mass, Inertia, gravity, delta_control, Matrix_states, Matrix_controls);
    [states_RK4(:,j), wdot(j)] = raunge_kutta_4(t_vec(j), Forces(:,j), Moments(:,j), states_RK4(:,j-1), mass, Inertia, I_inv, dt);
end

% Run Simulink
Simulator = sim('R21a.slx');

% Plot validation
plot_Testing(t_vec, states_RK4, Simulator , delta_control);