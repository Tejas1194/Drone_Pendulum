function run_simulation
    
    clear all; clc; close all;
    
    % --- 1. Simulation Parameters ---
    SIM_TIME = 15.0;           
    DT_PHYSICS = 1.0 / 500.0;  
    DT_OUTER_LOOP = 1.0 / 50.0;
    
    N_steps = round(SIM_TIME / DT_PHYSICS);
    outer_loop_decimation = round(DT_OUTER_LOOP / DT_PHYSICS);
    
    % --- 2. System Parameters ---
    params.m_F = 1.0;   
    params.m_B = 0.2;   
    params.l   = 0.5;   
    params.I   = 0.01;  % Phase 1 Inertia (Drone Only)
    params.g   = 9.81;  
    params.d   = 0.2;   
    
    % --- 3. Controller Gains ---
    gains.Kp_z = 2.0;
    gains.Kd_z = 1.5;
    
    gains.Kp_swing = 0.5;
    gains.lock_angle_deg = 75.0;   
    gains.theta_max_deg = 20.0;  
    
    gains.Kp_theta = 5.0;
    gains.Kd_theta = 1.0;
    
    % --- 4. Setpoints ---
    setpoints.z_desired = 2.0; 
    
    % --- 5. Initialization ---
    physics = DronePendulum(params);
    physics_locked = []; 
    
    gains.E_target = 1.05 * params.m_B * params.g * params.l * (1.0 - cosd(gains.lock_angle_deg));
    gains.theta_max = deg2rad(gains.theta_max_deg);
    
    % --- PRE-CALCULATION: PHASE 2 PHYSICS ---
    % Calculate the physics properties of the locked system so the controller doesn't crash
    beta_rad = deg2rad(gains.lock_angle_deg);
    total_mass = params.m_F + params.m_B;
    
    % 1. COM Offset (The "Backpack" Lever Arm)
    com_x_offset = (params.m_B * params.l * sin(beta_rad)) / total_mass;
    com_z_offset = -(params.m_B * params.l * cos(beta_rad)) / total_mass;
    
    % 2. Locked Inertia (The "Sledgehammer" Mass)
    dist_F_sq = com_x_offset^2 + com_z_offset^2;
    dist_B_sq = (params.l*sin(beta_rad) - com_x_offset)^2 + (-params.l*cos(beta_rad) - com_z_offset)^2;
    params.I_locked = params.I + (params.m_F * dist_F_sq) + (params.m_B * dist_B_sq);
    params.cx_locked = com_x_offset; 
    
    time_hist   = zeros(1, N_steps);
    state_hist  = zeros(8, N_steps); 
    T_hist      = zeros(1, N_steps);
    forces_hist = zeros(2, N_steps); 
    
    u_1 = (params.m_F + params.m_B) * params.g; 
    theta_desired = 0.0;
    
    is_locked = false; 
    lock_time = NaN; 
    
    disp('Starting simulation...');
    
    % --- 6. Main Simulation Loop ---
    for i = 1:N_steps
        
        t = (i - 1) * DT_PHYSICS;
        
        % A. GET STATE
        if ~is_locked
            state = physics.get_state();
            x = state(1); x_dot = state(2);
            z = state(3); z_dot = state(4);
            theta = state(5); theta_dot = state(6);
            beta  = state(7); beta_dot  = state(8);
        else
            state_locked = physics_locked.get_state();
            x = state_locked(1); x_dot = state_locked(2);
            z = state_locked(3); z_dot = state_locked(4);
            theta = state_locked(5); theta_dot = state_locked(6);
            beta = deg2rad(gains.lock_angle_deg); 
            beta_dot = 0;
        end
        
        % B. CHECK TRANSITION
        if ~is_locked && beta >= deg2rad(gains.lock_angle_deg)
            is_locked = true;
            lock_time = t; 
            
            fprintf('\n*** LOCK TRIGGERED at %.2fs (Angle: %.2f) ***\n', t, rad2deg(beta));
            
            current_kinematics = [x; x_dot; z; z_dot; theta; theta_dot];
            physics_locked = DronePendulum2(params, current_kinematics);
        end
        
        % C. OUTER LOOP CONTROLLERS
        if mod(i, outer_loop_decimation) == 0
            
            u_1 = altitude_controller(z, z_dot, theta, setpoints.z_desired, gains, params);
            
            if ~is_locked
                % MODE 1: SWING UP
                if t < 0.5 
                    theta_desired = deg2rad(5.0); 
                else
                    theta_desired = swing_up_controller(beta, beta_dot, gains, params);
                end
            else
                % MODE 2: LOCKED - SIMPLIFIED REQUEST
                % Just hold 0 angle and 2m height. No horizontal control.
                theta_desired = 0.0;
            end
        end
        
        % D. INNER LOOP (PITCH)
        % We pass is_locked and u_1 to handle the Feedforward Physics Fix
        u_2 = pitch_controller(theta, theta_dot, theta_desired, gains, params, is_locked, u_1);
        
        FF = (u_1 / 2.0) + (u_2 / (2.0 * params.d));
        FB = (u_1 / 2.0) - (u_2 / (2.0 * params.d));
        FF = max(0, FF);
        FB = max(0, FB);
        
        if ~is_locked
            physics.step(FF, FB, DT_PHYSICS);
            T_current = physics.get_tension();
        else
            physics_locked.step(FF, FB, DT_PHYSICS);
            T_current = 0; 
        end
        
        % E. LOGGING
        time_hist(i)   = t;
        if ~is_locked
            state_hist(:, i) = state;
        else
            state_hist(:, i) = [x; x_dot; z; z_dot; theta; theta_dot; beta; 0];
        end
        T_hist(i)      = T_current;
        forces_hist(:, i) = [FF; FB];
    end
    
    disp('Simulation finished.');
    save('simulation_results.mat', 'time_hist', 'state_hist', 'T_hist', 'forces_hist', 'params', 'gains', 'lock_time');
    disp('Results saved to simulation_results.mat');
end

% --- Controller Functions ---

function u_1 = altitude_controller(z, z_dot, theta, z_desired, gains, params)
    z_error = z_desired - z;
    z_dot_error = 0 - z_dot;
    z_accel_command = (gains.Kp_z * z_error) + (gains.Kd_z * z_dot_error);
    
    total_mass = params.m_F + params.m_B;
    F_vertical_command = total_mass * (params.g + z_accel_command);
    
    u_1 = F_vertical_command / max(cos(theta), 0.1); 
    max_thrust = 3.0 * total_mass * params.g; % Increased headroom for stability
    u_1 = min(u_1, max_thrust);
end

function theta_desired = swing_up_controller(beta, beta_dot, gains, params)
    PE = params.m_B * params.g * params.l * (1.0 - cos(beta));
    KE = 0.5 * params.m_B * (params.l^2) * (beta_dot^2);
    E_current = PE + KE;
   
    if E_current < gains.E_target
        theta_desired = gains.Kp_swing * beta_dot;
    else
        theta_desired = 0.0;
    end
    theta_desired = max(min(theta_desired, gains.theta_max), -gains.theta_max);
end

function u_2 = pitch_controller(theta, theta_dot, theta_desired, gains, params, is_locked, current_thrust)
    theta_error = theta_desired - theta;
    theta_dot_error = 0 - theta_dot;
    theta_accel_command = (gains.Kp_theta * theta_error) + (gains.Kd_theta * theta_dot_error);
    
    if ~is_locked
        % Phase 1: Standard Drone Physics
        u_2 = params.I * theta_accel_command;
    else
        % Phase 2: Rigid Body Physics (FIXED)
        % 1. Use Correct Inertia (System is heavier)
        u_2_feedback = params.I_locked * theta_accel_command;
        
        % 2. Feedforward for Center of Mass Offset
        % Cancels the torque created by thrust acting on the offset COM
        u_2_feedforward = current_thrust * params.cx_locked;
        
        u_2 = u_2_feedback + u_2_feedforward;
    end
end