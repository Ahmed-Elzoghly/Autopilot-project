function plot_Validation(t_vec_RK4, states_vec_RK4, simOut, delta_control)
    % Extract control input values for naming
    % aileron  = rad2deg(delta_control(1));
    % rudder   = rad2deg(delta_control(2));
    % elevator = rad2deg(delta_control(3));
    % thrust   = delta_control(4);

    % % Ensure filenames are safe
    % filename_prefix = sprintf('Ail_%d_Rud_%d_Elev_%d_Thr_%d', aileron, rudder, elevator, thrust);
    % filename_prefix = strrep(filename_prefix, '.', 'p'); % Replace decimal points

    % Load benchmark data
    benchmark_data = load('Benchmark_B747_FC5.mat');

    % Assume benchmark contains similar state variables
    benchmark_time = benchmark_data.time;  % Modify as per actual field name
    benchmark_states = benchmark_data.states;  % Modify as per actual field name

    labels = {'$\bf{u}$ (m/s)', '$\bf{\beta}$ (deg)', '$\bf{\alpha}$ (deg)', ...
              '$\bf{p}$ (rad/s)', '$\bf{q}$ (rad/s)', '$\bf{r}$ (rad/s)', ...
              '$\bf{\phi}$ ($^\circ$)', '$\bf{\theta}$ ($^\circ$)', '$\bf{\psi}$ ($^\circ$)', ...
              '$\bf{x}$ (m)', '$\bf{y}$ (m)', '$\bf{z}$ (m)'};

    % Convert Euler angles to degrees for RK4 results

    states_vec_RK4(4, :) = rad2deg(states_vec_RK4(7, :)); % p
    states_vec_RK4(5, :) = rad2deg(states_vec_RK4(8, :)); % q
    states_vec_RK4(6, :) = rad2deg(states_vec_RK4(9, :)); % r

    states_vec_RK4(7, :) = rad2deg(states_vec_RK4(7, :)); % phi
    states_vec_RK4(8, :) = rad2deg(states_vec_RK4(8, :)); % theta
    states_vec_RK4(9, :) = rad2deg(states_vec_RK4(9, :)); % psi

    % Plot all 12 states in one figure with benchmark data
    figure;
    set(gcf, 'Units', 'normalized', 'OuterPosition', [0 0 1 1]); % Fullscreen mode
    sgtitle('$\bf{State Variables Comparison}$', 'Interpreter', 'latex', 'FontSize', 16);

    for i = 1:12
        subplot(4, 3, i); hold on;
        
        plot(t_vec_RK4, states_vec_RK4(i, :), 'b', 'LineWidth', 1.5);
        plot(simOut.tout, simOut.get(i).data, '--r', 'LineWidth', 1.5); % Simulink data
        plot(benchmark_time, benchmark_states(i, :), '-.g', 'LineWidth', 1.5); % Benchmark data

        title(labels{i}, 'Interpreter', 'latex', 'FontSize', 12);
        xlabel('$\bf{t}$ (sec)', 'Interpreter', 'latex', 'FontSize', 10);
        ylabel(labels{i}, 'Interpreter', 'latex', 'FontSize', 10);
        
        legend('RK4', 'Simulink', 'Benchmark', 'Interpreter', 'latex', 'FontSize', 8);
        grid on;
    end

    % Save the comparison figure
    % saveas(gcf, sprintf('%s_Comparison.svg', filename_prefix));
end
