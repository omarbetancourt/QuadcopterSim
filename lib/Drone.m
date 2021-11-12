classdef Drone < handle
    %% MEMBERS
    properties % of Dynamics
        g;    % gravitiy         [m/s^2]
        t;    % time             [s]
        dt;   % time step size   [s]
        tf;   % final time       [s]
        
        m;    % mass of drone    [kg]
        l;    % drone arm length [m]
        I;    % Mass-mom. matrix [kg-m^2]
        
        x;    % state vector     [X-pos, Y-pos, Z-pos, dX, dY, dZ, phi, theta, psi, p, q, r]
        r;    % position vector  [X-pos, Y-pos, Z-pos]
        dr;   % velocity vector  [dX, dY, dZ]
        euler;% euler angles     [phi, theta, psi]
        w;    % drone ang. vel.  [p, q, r]
        
        dx;   % change in state vector
        
        T;    % total thrust     [N]
        u;    % input vector     [T_sum, M1, M2, M3]
        
        M;    % Moment vector    [M1, M2, M3]
    end
        
    properties 
        des_state   % desired state vector
        r_des       % desired position vector   [X_des, Y_des, Z_des]
        dr_des      % desired velocity vector   [dX_des, dY_des, dZ_des]
        w_des       % desire ang. vel vector    [p_des, q_des, r_des]
        yaw_des     % desired yaw angle         [psi_des]
         
        acc_des
        acc_c
        xacc_c     
        yacc_c        
        zacc_c
        
        phi_des     % desired roll angle
        phi_err     % phi_actual - phi_des
        
        theta_des   % desired pitch angle
        theta_err   % theta_actual - theta_des     
        
        psi_des     % desired yaw angle
        psi_err     % psi_actual - psi_des       
        
        xpos_des    % desired x-pos
        xpos_err
        
        ypos_des    % desired y-pos
        ypos_err
        
        zpos_des    % desired z-pos
        zpos_err
        
        % Roll angle phi gains
        kP_phi
        kI_phi
        kD_phi
        
        % Pitch angle theta gains
        kP_theta
        kI_theta
        kD_theta
        
        % Yaw angle psi gains
        kP_psi
        kI_psi
        kD_psi
        
        % x-position gains
        kP_x
        kI_x
        kD_x
        
        % y-position gains
        kP_y
        kI_y
        kD_y
        
        % z-position gains
        kP_z
        kI_z
        kD_z
    end
    
    properties
        dmt
        
        vel_max
    end
    
    %% METHODS
    methods
        %% CONSTRUCTOR
        function obj = Drone(params, initStates, initInputs, gains, simTime)
            obj.g  = 9.81;
            obj.t  = 0.0;
            obj.dt = 0.001;
            obj.tf = simTime;
            
            obj.m = params('mass');
            obj.l = params('armLength');
            obj.I = [params('Ixx'), 0,             0; %% I matrix
                     0,             params('Iyy'), 0; 
                     0,             0,             params('Izz')];
            
            obj.des_state = zeros(13,1);
            obj.r_des   = obj.des_state(1:3);
            obj.dr_des  = obj.des_state(4:6);
            obj.acc_des = obj.des_state(7:9);
            obj.w_des   = obj.des_state(10:12);
            obj.yaw_des = obj.des_state(13);
            
            obj.x     = initStates;
            obj.r     = obj.x(1:3);
            obj.dr    = obj.x(4:6);
            obj.euler = obj.x(7:9);
            obj.w     = obj.x(10:12);
            
            obj.dx  = zeros(12,1); 
            obj.u   = initInputs;
            obj.T   = obj.u(1);
            obj.M   = obj.u(2:4);
            
            obj.dmt = 0.0;
            obj.vel_max = 10.0;
            
            obj.acc_c   = zeros(3,1);
            obj.xacc_c  = obj.acc_c(1);
            obj.yacc_c  = obj.acc_c(2);
            obj.zacc_c  = obj.acc_c(3);
      
            obj.phi_des = 0.0;
            obj.phi_err = 0.0;     

            obj.theta_des = 0.0;
            obj.theta_err = 0.0;        

            obj.psi_des = 0.0;
            obj.psi_err = 0.0;
            
            obj.xpos_des = 0.0;
            obj.xpos_err = 0.0;
            
            obj.ypos_des = 0.0;
            obj.ypos_err = 0.0;
            
            obj.zpos_des = 0.0;
            obj.zpos_err = 0.0;      

            obj.kP_phi = gains('P_phi');
            obj.kI_phi = gains('I_phi');
            obj.kD_phi = gains('D_phi');

            obj.kP_theta = gains('P_theta');
            obj.kI_theta = gains('I_theta');
            obj.kD_theta = gains('D_theta');

            obj.kP_psi = gains('P_psi');
            obj.kI_psi = gains('I_psi');
            obj.kD_psi = gains('D_psi');
            
            obj.kP_x = gains('P_x');
            obj.kI_x = gains('I_x');
            obj.kD_x = gains('D_x');
            
            obj.kP_y = gains('P_y');
            obj.kI_y = gains('I_y');
            obj.kD_y = gains('D_y');
            
            obj.kP_z = gains('P_z');
            obj.kI_z = gains('I_z');
            obj.kD_z = gains('D_z');
        end
        
        function state = GetState(obj)  % Getter function for state variable
            state = obj.x;
        end
               
        function obj = EvalEOM(obj)     % Evaluating Equations of motion
            bRi = RPY2Rot(obj.euler);   % Roll Pitch Yaw to Rotation
            R = bRi';                   % Body-fixed frame to inertial reference frame
            obj.dx(1:3) = obj.dr;       % Set first 3 elements of dx to dr
            
            %Newton's Equation of motion - Translational Acceleration (Eq 3)
            obj.dx(4:6) = 1 / obj.m * ([0; 0; -obj.m*obj.g] + R * obj.T * [0; 0; 1]);
            
            % Rotational Motions - Angular Velocity
            phi = obj.euler(1); theta = obj.euler(2);
            obj.dx(7:9) =[ 1  sin(phi)*tan(theta)  cos(phi)*tan(theta);
                           0  cos(phi)             -sin(phi);
                           0  sin(phi)*sec(theta)  cos(phi)*sec(theta)] * obj.w; % This gives phi_dot, theta_dot, psi_dot (angular velocities)
                       
            % Angular Accelerations
            obj.dx(10:12) = (obj.I) \ (obj.M - cross(obj.w, obj.I *obj.w));
            
        end
        
        function obj = UpdateState(obj)
            obj.t = obj.t + obj.dt         % Propogate 1 time setp
            
            obj.EvalEOM();                  % This function updates dx
            
%             norm_vel = obj.x(4:6)/norm(obj.x(4:6));
%             if norm(obj.x(4:6)) >= obj.vel_max
%                 obj.x(4:6) = norm_vel*obj.vel_max;
%                 obj.dx(1:3) = obj.x(4:6);
%             end
            % x_new = x_current + del_x/del_t * del_t
            obj.x = obj.x + obj.dx*obj.dt;  % Euler method updates state vector x
            
            % Update other vectors
            obj.r = obj.x(1:3);
            obj.dr = obj.x(4:6);
            obj.euler = obj.x(7:9);
            obj.w = obj.x(10:12);
        end
        
        function obj = PositionCtrl(obj, refSig)
            % Desired position (imported info from refSig in main) will
            % ultimatly come from Tarek's algorithm (nhat)
            % How to implement direction to actual coordinates?
            obj.r_des = [refSig(1), refSig(2), refSig(3)];
            
            obj.dr_des = zeros(3,1);
            obj.acc_des = zeros(3,1);
            
            kD = [obj.kD_x; obj.kD_y; obj.kD_z];
            kP = [obj.kP_x; obj.kP_y; obj.kP_z];
            
            obj.acc_c = obj.acc_des + kD .* (obj.dr_des - obj.dr) + kP .* (obj.r_des - obj.r); 
            
            obj.u(1) = obj.m*obj.g + obj.m*(obj.acc_c(3));
            
            obj.T = obj.u(1);   
        end
        
        function obj = AttitudeCtrl(obj)
            obj.phi_des   = 1/obj.g*(obj.acc_c(1)*sin(obj.yaw_des) - obj.acc_c(2)*cos(obj.yaw_des));
            obj.theta_des = 1/obj.g*(obj.acc_c(1)*cos(obj.yaw_des) + obj.acc_c(2)*sin(obj.yaw_des));
            
            angle_des = [obj.phi_des; obj.theta_des; obj.yaw_des];
            
            kD = [obj.kD_phi; obj.kD_theta; obj.kD_psi];
            kP = [obj.kP_phi; obj.kP_theta; obj.kP_psi];
            
            obj.u(2:4) = kD .* (obj.w_des - obj.w) + kP .* (angle_des - obj.euler);
 
            obj.M = obj.u(2:4);
        end
        
        function obj = DistTar(obj, tar)
            obj.dmt = norm(obj.r - tar.GetPos);
        end
        
        function DistanceToTarget = GetDmt(obj)
            DistanceToTarget = obj.dmt;
        end
    end
    
end