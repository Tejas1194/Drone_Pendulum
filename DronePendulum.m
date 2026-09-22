classdef DronePendulum < handle
    % DronePendulum Simulates the 2D dynamics of a drone with a pendulum.
    %
    % This class encapsulates the physics of the system. The main simulation
    % loop will create an object of this class and call the 'step' method.

    properties
        % --- System Parameters ---
        m_F;  % Mass of the drone frame (kg)
        m_B;  % Mass of the pendulum bob (kg)
        l;    % Length of the pendulum link (m)
        I;    % Moment of inertia of the drone (kg*m^2)
        g;    % Acceleration due to gravity (m/s^2)
        d;    % Distance from COM to motors (m)
        
        % --- Simulation State ---
        state; % 8x1 state vector [x, x_dot, z, z_dot, theta, theta_dot, beta, beta_dot]
        time;  % Current simulation time (s)
        
        % --- Internal Tension (for logging) ---
        T;     % Last calculated tension (N)
    end
    
    methods
        function obj = DronePendulum(params)
            % Constructor: Initialize the system parameters and state.
            obj.m_F = params.m_F;
            obj.m_B = params.m_B;
            obj.l   = params.l;
            obj.I   = params.I;
            obj.g   = params.g;
            obj.d   = params.d;
            
            % Initial state: [x, x_dot, z, z_dot, theta, theta_dot, beta, beta_dot]
            % Start at origin (x=0, z=0), at rest, level, with pendulum hanging.
            obj.state = zeros(8, 1);
            obj.time  = 0;
            obj.T     = (obj.m_F + obj.m_B) * obj.g; % Initial tension at hover
        end
        
        function step(obj, FF, FB, dt)
            % It takes motor forces and a time step (dt) and updates the state
            % using the 4-DOF non-linear equations of motion.
            
            % --- 1. Get current state variables for readability ---
            x       = obj.state(1);
            x_dot   = obj.state(2);
            z       = obj.state(3);
            z_dot   = obj.state(4);
            theta   = obj.state(5);
            theta_dot = obj.state(6);
            beta    = obj.state(7);
            beta_dot  = obj.state(8);
            
            % --- 2. Pre-calculate Trig Terms ---
            s_t = sin(theta);
            c_t = cos(theta);
            s_b = sin(beta);
            c_b = cos(beta);
            s_tb = sin(theta + beta);
            c_tb = cos(theta + beta);
            
            % --- 3. Solve for Accelerations (ddx, ddz, ddtheta, ddbeta) ---
            
            % We have 5 coupled EOMs and 5 unknowns (ddx, ddz, ddtheta, ddbeta, T).
            % EOM 1 (Drone X):   m_F*ddx = -(FF+FB)*s_t + T*s_tb
            % EOM 2 (Drone Z):   m_F*ddz = (FF+FB)*c_t - T*c_tb - m_F*g
            % EOM 3 (Drone Q):   I*ddtheta = (FF-FB)*d  (Decoupled, T has no torque at COM)
            % EOM 4 (Pendulum Tan): l*ddbeta = l*ddtheta - g*s_tb - ddx*c_tb - ddz*s_tb
            % EOM 5 (Pendulum Rad): T = m_B*g*c_tb + m_B*(ddz*c_tb - ddx*s_tb) + m_B*l*(beta_dot - theta_dot)^2
            %
            % Note: EOM 5 is the full radial EOM on the bob, including
            % Centrifugal (m*l*theta_dot^2), Coriolis (-2*m*l*theta_dot*beta_dot)
            % and bob relative accel (m*l*beta_dot^2). These combine to m*l*(beta_dot - theta_dot)^2.

            % --- Strategy ---
            % 1. Solve EOM 3 for ddtheta (it's independent).
            % 2. Substitute EOM 5 (T) into EOM 1 (Drone X) to get (Eq A).
            % 3. Substitute EOM 5 (T) into EOM 2 (Drone Z) to get (Eq B).
            % 4. (Eq A) and (Eq B) form a 2x2 linear system M*[ddx; ddz] = C.
            % 5. Solve the 2x2 system for ddx and ddz.
            % 6. Substitute ddx, ddz, ddtheta into EOM 4 to solve for ddbeta.
            % 7. Substitute ddx, ddz into EOM 5 to solve for T (for logging).
            
            % Step 1: Solve for ddtheta
            ddtheta = (FF - FB) * obj.d / obj.I;

            % Step 2 & 3: Define the 2x2 system M*[ddx; ddz] = C
            
            M = [ (obj.m_F + obj.m_B*s_tb^2) , (-obj.m_B*s_tb*c_tb);
                  (-obj.m_B*s_tb*c_tb)      , (obj.m_F + obj.m_B*c_tb^2) ];
            
            % Pre-calculate the shared velocity-dependent term
            vel_term =  (beta_dot - theta_dot)^2;
              
            C = [ -(FF+FB)*s_t + obj.m_B*s_tb*c_tb*obj.g + obj.m_B*obj.l*s_tb * vel_term;
                  (FF+FB)*c_t  - obj.m_B*c_tb^2*obj.g - obj.m_B*obj.l* c_tb * vel_term - obj.m_F*obj.g];
            
            % Step 5: Solve for ddx, ddz
            % Check if M is invertible (it can become singular if m_F = 0)
            if det(M) < 1e-6
                % Handle singularity, e.g., by not accelerating
                accels_xz = [0; 0];
            else
                accels_xz = M \ C;
            end
            ddx = accels_xz(1);
            ddz = accels_xz(2);
            
            % Step 6: Solve for ddbeta using EOM 4
            ddbeta = (-1/obj.l) * (-obj.l*ddtheta + obj.g*s_tb + ddx*c_tb + ddz*s_tb);

            % Step 7: Solve for T (for logging) using EOM 5
            obj.T = obj.m_B*(vel_term*obj.l - ddx*s_tb +(ddz+obj.g)*c_tb);
            
            % --- 4. Integrate to update state (Forward Euler) ---
            % This is a simple but effective numerical integration method.
            obj.state(1) = obj.state(1) + obj.state(2) * dt; % x = x + x_dot * dt
            obj.state(2) = obj.state(2) + ddx * dt;           % x_dot = x_dot + ddx * dt
            
            obj.state(3) = obj.state(3) + obj.state(4) * dt; % z = z + z_dot * dt
            obj.state(4) = obj.state(4) + ddz * dt;           % z_dot = z_dot + ddz * dt
            
            obj.state(5) = obj.state(5) + obj.state(6) * dt; % theta = theta + theta_dot * dt
            obj.state(6) = obj.state(6) + ddtheta * dt;       % theta_dot = theta_dot + ddtheta * dt
            
            obj.state(7) = obj.state(7) + obj.state(8) * dt; % beta = beta + beta_dot * dt
            obj.state(8) = obj.state(8) + ddbeta * dt;        % beta_dot = beta_dot + ddbeta * dt
            
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
        
        function T = get_tension(obj)
            % Helper function to get the last calculated tension.
            T = obj.T;
        end
    end
end

