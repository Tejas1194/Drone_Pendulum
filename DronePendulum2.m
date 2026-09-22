classdef DronePendulum2 < handle
    % DronePendulum2 Simulates the 2D dynamics of the LOCKED rigid body.
    %
    % This class models the system as a single rigid body after the 
    % pendulum has been locked at beta = 75 degrees.
    %
    % The state is 6-DOF: [x, x_dot, z, z_dot, theta, theta_dot]
    % where (x, z, theta) are the coordinates of the *pivot* (original drone COM).

    properties
        % --- System Parameters ---
        params; % Struct containing m_F, m_B, l, I, g, d
        
        % --- Simulation State ---
        state; % 6x1 state vector [x, x_dot, z, z_dot, theta, theta_dot]
        time;  % Current simulation time (s)
        
        % --- Pre-calculated Properties ---
        M_total; % Total mass (m_F + m_B)
        c_x;     % Body-frame x-position of COM (from pivot)
        c_z;     % Body-frame z-position of COM (from pivot)
        I_com;   % Total Moment of Inertia about the new combined COM
        beta_lock_rad; % Pendulum lock angle in radians
    end
    
    methods
        function obj = DronePendulum2(params, initial_state_6dof)
            % Constructor: Initialize the locked system.
            obj.params = params;
            obj.time = 0;
            
            % Set the lock angle (hardcoded as 75 deg)
            beta_lock = deg2rad(75.0);
            obj.beta_lock_rad = beta_lock;

            % --- Calculate Combined Rigid Body Properties ---
            
            obj.M_total = params.m_F + params.m_B;
            lambda = params.m_B / obj.M_total;

            % 1. Find new COM position relative to pivot (in body frame)
            % (c_x, c_z) is the vector from the pivot to the new COM
            obj.c_x = lambda * params.l * sin(beta_lock);
            obj.c_z = -lambda * params.l * cos(beta_lock);

            % 2. Find new Moment of Inertia about the new COM
            %    using the Parallel Axis Theorem.
            
            % Distance from pivot (drone COM) to new COM, squared
            dist_F_to_COM_sq = obj.c_x^2 + obj.c_z^2;
            
            % Distance from bob to new COM, squared
            % Bob position (r_B) = [l*sin(beta), -l*cos(beta)]
            % COM position (r_C) = [c_x, c_z]
            % dist^2 = (r_B_x - c_x)^2 + (r_B_z - c_z)^2
            dist_B_to_COM_sq = (params.l * sin(beta_lock) - obj.c_x)^2 + ...
                               (-params.l * cos(beta_lock) - obj.c_z)^2;

            % I_com = I_drone_about_com + I_bob_about_com
            I_drone_com = params.I + params.m_F * dist_F_to_COM_sq;
            I_bob_com   = params.m_B * dist_B_to_COM_sq; % Bob is point mass
            obj.I_com = I_drone_com + I_bob_com;
            
            
            % --- Set Initial State ---
            if nargin > 1
                % If an initial state is provided (e.g., from phase 1)
                obj.state = initial_state_6dof;
            else
                % Default initial state
                obj.state = zeros(6, 1);
            end
        end
        
        function step(obj, FF, FB, dt)
            % This is the core of the Phase 2 simulation.
            % It solves the 3-DOF equations of motion for the
            % combined rigid body, based on the new COM.
            
            % --- 1. Get current state variables ---
            x       = obj.state(1);
            x_dot   = obj.state(2);
            z       = obj.state(3);
            z_dot   = obj.state(4);
            theta   = obj.state(5);
            theta_dot = obj.state(6);
            
            % --- 2. Get parameters ---
            M_total = obj.M_total;
            I_com   = obj.I_com;
            c_x     = obj.c_x;
            m_B     = obj.params.m_B;
            l       = obj.params.l;
            g       = obj.params.g;
            d       = obj.params.d;
            beta_lock = obj.beta_lock_rad;

            % --- 3. Pre-calculate Trig Terms ---
            s_t  = sin(theta);
            c_t  = cos(theta);
            s_tb = sin(theta + beta_lock);
            c_tb = cos(theta + beta_lock);

            % --- 4. Define External Forces/Torques ---
            T_F = FF + FB; % Total Thrust
            tau_m = (FF - FB) * d; % Motor Torque (RHR, CCW positive)
            
            % --- 5. Solve for Accelerations (COM-based, RHR) ---
            
            % These EOMs are decoupled. We solve for ddtheta first,
            % then use it to solve for ddx and ddz.
            
            % --- Eq C: Torque about COM (RHR) ---
            % I_com * ddtheta = tau_m + tau_thrust_offset
            % tau_m = (FF - FB) * d  (Positive CCW)
            % tau_thrust_offset = -(FF + FB) * c_x (Negative CW)
            tau_net = tau_m - T_F * c_x; 
            ddtheta = tau_net / I_com;
            
            % --- Eq A: Horizontal Force (X-direction) ---
            % Solves sum(F_x) = M_total * a_com_x
            % M_total * a_com_x = -T_F*s_t
            % M_total * (ddx + m_B/M_total*l*c_tb*ddtheta - m_B/M_total*l*s_tb*theta_dot^2) = -T_F*s_t
            % M_total*ddx + m_B*l*c_tb*ddtheta - m_B*l*s_tb*theta_dot^2 = -T_F*s_t
            % M_total*ddx = -T_F*s_t + m_B*l*s_tb*theta_dot^2 - m_B*l*c_tb*ddtheta
            
            F_net_x = -T_F * s_t + m_B * l * s_tb * (theta_dot^2);
            ddx = (F_net_x - m_B * l * c_tb * ddtheta) / M_total;

            
            % --- Eq B: Vertical Force (Z-direction) ---
            % Solves sum(F_z) = M_total * a_com_z
            % M_total * a_com_z = T_F*c_t - M_total*g
            % M_total * (ddz + m_B/M_total*l*s_tb*ddtheta + m_B/M_total*l*c_tb*theta_dot^2) = T_F*c_t - M_total*g
            % M_total*ddz + m_B*l*s_tb*ddtheta + m_B*l*c_tb*theta_dot^2 = T_F*c_t - M_total*g
            % M_total*ddz = T_F*c_t - M_total*g - m_B*l*c_tb*theta_dot^2 - m_B*l*s_tb*ddtheta
            
            F_net_z = T_F * c_t - M_total * g - m_B * l * c_tb * (theta_dot^2);
            ddz = (F_net_z - m_B * l * s_tb * ddtheta) / M_total;

            
            % --- 6. Integrate to update state (Forward Euler) ---
            obj.state(1) = obj.state(1) + obj.state(2) * dt; % x
            obj.state(2) = obj.state(2) + ddx * dt;           % x_dot
            
            obj.state(3) = obj.state(3) + obj.state(4) * dt; % z
            obj.state(4) = obj.state(4) + ddz * dt;           % z_dot
            
            obj.state(5) = obj.state(5) + obj.state(6) * dt; % theta
            obj.state(6) = obj.state(6) + ddtheta * dt;       % theta_dot
            
            obj.time = obj.time + dt;
        end
        
        function s = get_state(obj)
            % Helper function to get the current state vector.
            s = obj.state;
        end
        
        function t = get_time(obj)
            % Helper function to get the current simulation time.
            t = obj.time;
        end
    end
end