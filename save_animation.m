function animate_simulation
    % Loads data from 'simulation_results.mat' and creates an animation
    % of the drone and pendulum, saving it to an MP4 video file.
    
    close all; % Close existing figures
    
    % --- 1. Load Data ---
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
    
    x       = s(1, :);
    z       = s(3, :);
    theta   = s(5, :);
    beta    = s(7, :);
    l       = params.l;
    d       = params.d; % Drone motor distance
    
    % --- 2. Animation Setup ---
    
    % Downsample for faster animation/video generation.
    % Physics is 500Hz. skipping 25 frames = 20Hz effective video rate.
    frame_skip = 25; 
    
    t       = t(1:frame_skip:end);
    x       = x(1:frame_skip:end);
    z       = z(1:frame_skip:end);
    theta   = theta(1:frame_skip:end);
    beta    = beta(1:frame_skip:end);

    % Create figure window
    % Set a fixed size to ensure consistent video resolution
    figure('Name', 'Drone Pendulum Animation', 'NumberTitle', 'off', ...
           'Position', [100, 100, 800, 600], 'Color', 'w'); 
    ax = gca;
    hold(ax, 'on');
    axis(ax, 'equal');
    grid(ax, 'on');
    
    % Set stable axis limits. Adjust these if your drone moves a lot.
    axis(ax, [-10, 2, 0, 4]);
    xlabel(ax, 'Horizontal Position (x) [m]');
    ylabel(ax, 'Vertical Position (z) [m]');
    
    % Initialize plot handles
    drone_body_line = plot(ax, NaN, NaN, 'b-', 'LineWidth', 3);
    pendulum_line   = plot(ax, NaN, NaN, 'r-', 'LineWidth', 1.5);
    pendulum_bob    = plot(ax, NaN, NaN, 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 10);
    title_handle    = title(ax, 'Time: 0.00 s');
    
    % --- VIDEO WRITER SETUP ---
    video_filename = 'drone_pendulum_animation.mp4';
    v = VideoWriter(video_filename, 'MPEG-4');
    v.FrameRate = 20; % 20 fps matches the downsampling (500Hz / 25 = 20Hz)
    v.Quality = 95;   % High quality
    open(v);
    disp(['Recording video to ', video_filename, '...']);
    
    % --- 3. Animation Loop ---
    for i = 1:length(t)
        
        % Get current state
        xi = x(i);
        zi = z(i);
        theta_i = theta(i);
        beta_i = beta(i);
        
        % --- Calculate Positions ---
        
        % Drone body (line from back motor to front motor)
        front_motor_x = xi + d * cos(theta_i);
        front_motor_z = zi + d * sin(theta_i);
        back_motor_x  = xi - d * cos(theta_i);
        back_motor_z  = zi - d * sin(theta_i);
        
        % Pendulum bob
        bob_x = xi + l * sin(theta_i + beta_i);
        bob_z = zi - l * cos(theta_i + beta_i);
        
        % --- Update Plots ---
        
        % Update drone body
        set(drone_body_line, 'XData', [back_motor_x, front_motor_x], ...
                             'YData', [back_motor_z, front_motor_z]);
        
        % Update pendulum line (from pivot to bob)
        set(pendulum_line, 'XData', [xi, bob_x], ...
                           'YData', [zi, bob_z]);
        
        % Update pendulum bob
        set(pendulum_bob, 'XData', bob_x, 'YData', bob_z);
        
        % Update title with current time
        set(title_handle, 'String', sprintf('Time: %.2f s', t(i)));
        
        % Force a draw update so getframe captures the fresh data
        drawnow;
        
        % --- Capture Frame for Video ---
        frame = getframe(gcf);
        writeVideo(v, frame);
        
        % (Optional) Remove pause if you want it to render as fast as possible
        % pause(0.01); 
    end
    
    % --- Cleanup ---
    close(v);
    disp(['Animation finished. Saved to ', video_filename]);

end