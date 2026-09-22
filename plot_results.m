function plot_results
    % Loads and plots the simulation results.
    
    close all;
    
    % Load data
    try
        data = load('simulation_results.mat');
    catch
        disp('Could not find simulation_results.mat.');
        disp('Please run run_simulation.m first.');
        return;
    end

    % Unpack data
    t       = data.time_hist;
    s       = data.state_hist;
    params  = data.params;
    gains   = data.gains;
    lock_time = data.lock_time;
    
    x         = s(1, :);
    x_dot     = s(2, :);
    z         = s(3, :);
    z_dot     = s(4, :);
    theta     = rad2deg(s(5, :)); % Convert to degrees for plotting
    theta_dot = rad2deg(s(6, :));
    beta      = rad2deg(s(7, :));
    beta_dot  = rad2deg(s(8, :));

    % Calculate energy
    PE = params.m_B * params.g * params.l * (1.0 - cosd(beta));
    KE = 0.5 * params.m_B * (params.l^2) * (s(8, :).^2);
    E_total = PE + KE;
    E_target_line = ones(size(t)) * (params.m_B * params.g * params.l * (1.0 - cosd(gains.lock_angle_deg)));
    
    % --- Create Plots ---
    
    figure('Name', 'Drone Pendulum (Two-Phase)', 'NumberTitle', 'off', 'WindowState', 'maximized');
    sgtitle('Drone-Pendulum Simulation ');
    
    % Plot 1: Pendulum Angle
    subplot(3, 2, 1);
    plot(t, beta, 'b', 'LineWidth', 1.5);
    hold on;
    plot(t, ones(size(t)) * gains.lock_angle_deg, 'r--', 'LineWidth', 1);
    plot(t, -ones(size(t)) * gains.lock_angle_deg, 'r--', 'LineWidth', 1);
    plot_lock_line(lock_time);
    title('Pendulum Angle (\beta)');
    xlabel('Time (s)');
    ylabel('Angle (deg)');
    legend('\beta', 'Target');
    grid on;
    
    % Plot 2: Pendulum Energy
    subplot(3, 2, 2);
    plot(t, E_total, 'g', 'LineWidth', 1.5);
    hold on;
    plot(t, E_target_line, 'r--', 'LineWidth', 1);
    plot_lock_line(lock_time);
    title('Pendulum Energy');
    xlabel('Time (s)');
    ylabel('Energy (J)');
    legend('E_{total}', 'E_{target}');
    grid on;

    % Plot 3: Drone Altitude
    subplot(3, 2, 3);
    plot(t, z, 'b', 'LineWidth', 1.5);
    hold on;
    plot_lock_line(lock_time);
    title('Drone Altitude (z)');
    xlabel('Time (s)');
    ylabel('Position (m)');
    grid on;

    % Plot 4: Drone Pitch Angle
    subplot(3, 2, 4);
    plot(t, theta, 'm', 'LineWidth', 1.5);
    hold on;
    plot_lock_line(lock_time);
    title('Drone Pitch Angle (\theta)');
    xlabel('Time (s)');
    ylabel('Angle (deg)');
    grid on;
    
    % Plot 5: Drone Position (X)
    subplot(3, 2, 5);
    plot(t, x, 'b', 'LineWidth', 1.5);
    hold on;
    plot_lock_line(lock_time);
    title('Drone Horizontal Position (x)');
    xlabel('Time (s)');
    ylabel('Position (m)');
    grid on;
    
    % Plot 6: Motor Forces
    subplot(3, 2, 6);
    plot(t, data.forces_hist(1, :), 'r-', 'LineWidth', 1.5);
    hold on;
    plot(t, data.forces_hist(2, :), 'b-', 'LineWidth', 1.5);
    plot_lock_line(lock_time);
    title('Motor Forces');
    xlabel('Time (s)');
    ylabel('Force (N)');
    legend('F_F (Front)', 'F_B (Back)');
    grid on;

end

function plot_lock_line(lock_time)
    % Helper function to draw a vertical line at the lock time
    if isfinite(lock_time)
        yl = ylim;
        line([lock_time lock_time], yl, 'Color', 'w', 'LineStyle', ':', 'LineWidth', 1.5);
    end
end