%% ========================== Plotting Function ==========================
function plotValidation(t_vec_RK4, states_vec_RK4, simOut)
    % Function to compare simulation results from RK4 and Simulink
    figure_titles = {'Linear Velocities Validation', ...
                     'Angular Velocities Validation', ...
                     'Euler Angles Validation', ...
                     'Position Validation'};

    labels = {'$\bf{u}$ (m/s)', '$\bf{v}$ (m/s)', '$\bf{w}$ (m/s)', ...
              '$\bf{p}$ (rad/s)', '$\bf{q}$ (rad/s)', '$\bf{r}$ (rad/s)', ...
              '$\bf{\phi}$ ($^\circ$)', '$\bf{\theta}$ ($^\circ$)', '$\bf{\psi}$ ($^\circ$)', ...
              '$\bf{x}$ (m)', '$\bf{y}$ (m)', '$\bf{z}$ (m)'};

    % Convert phi, theta, and psi from radians to degrees in states_vec_RK4
    states_vec_RK4(7, :) = rad2deg(states_vec_RK4(7, :)); % phi
    states_vec_RK4(8, :) = rad2deg(states_vec_RK4(8, :)); % theta
    states_vec_RK4(9, :) = rad2deg(states_vec_RK4(9, :)); % psi

    simData = {simOut.u_simulink, simOut.v_simulink, simOut.w_simulink, ...
               simOut.p_simulink, simOut.q_simulink, simOut.r_simulink, ...
               simOut.phi_simulink, simOut.theta_simulink, simOut.psi_simulink, ...
               simOut.x_simulink, simOut.y_simulink, simOut.z_simulink};

    for j = 1:4  % Loop for 4 figures
        figure;
        sgtitle(['$\bf{' figure_titles{j} '}$'], 'Interpreter', 'latex', 'FontSize', 16);
        
        for i = 1:3
            idx = (j-1)*3 + i;
            subplot(3,1,i); hold on;
            
            plot(t_vec_RK4, states_vec_RK4(idx, :), 'b', 'LineWidth', 1.5);
            plot(simData{idx}.time, simData{idx}.data, '--r', 'LineWidth', 1.5);
            
            title(labels{idx}, 'Interpreter', 'latex', 'FontSize', 14);
            xlabel('$\bf{t}$ (sec)', 'Interpreter', 'latex', 'FontSize', 14);
            ylabel(labels{idx}, 'Interpreter', 'latex', 'FontSize', 14);
            
            legend(['RK4 ' labels{idx}], ['Simulink ' labels{idx}], ...
                   'Interpreter', 'latex', 'FontSize', 12);
            grid on;
        end
    end
end