classdef Drone < handle
    
%% MEMBERS    
    properties
        g               % gravity acceleration
        t               % start time
        dt              % derivative with respect to time
        tf              % final time
        
        %% Initial Condition

        x              % position
        v              % velocity
        R              % actitude
        omega          % angular velocity

        psi_d
        dot_psi_d
        e_f
        

        %% Body Frame

        euler

        phi
        psi
        theta

        %% Parameter
        m              % mass 
        b              % gyro bias
        M              % inertial matrix

        % Measurements

        omega_g
        v_i

        

       
        
        %% State
        s
        stateR
        dotS
        dotR

        %% Input
        u
        f
        tau


        
    end

    %% Control Problem

    properties
        % Position-tracking
        
        x_des       % desired position
        x_err       % error position
        x_err_prev
        x_err_sum

        v_des       % desired velocity
        v_err       % error position
        v_err_prev
        v_err_sum

        dotV_des    % desired acceleration
        dotV_err    % error acceleration
        dotV_err_prev
        dotV_err_sum


        R_des % desired attitude
        dotR_des
        omega_des   % desired omega
    end
      
    properties
        % Position Controller

        k
        k_x
        K_f
    end

    properties
        %control law
        
        T
        dotE_f
        eta
        ddx_d
    end

    properties
        % Control alttitude parameters
        alpha_1
        alpha_2
        lambda_c        
        K_c
        dotOmega_des
        J
    end

    properties
        %Bias Observer
        Lambda_i
        gamma_f
        v_f_i
        hat_b
        dot_bar_b
        hat_omega
        dot_v_f_i

        %Filter
        A
        vartheta_1
        vartheta_2
        dot_vartheta_1
        dot_vartheta_2

        % Attitude controller
        omega_r
        dot_hat_omega_r
        dot_v_i
        dot_v_d_i

    end

    properties
        %Inertial Vectors
        r_1
        r_2
        r_3
        k_i
        r_i
    end

    properties
        % vector alignment error
        v_d_i
        epsilon
        z
        dotEpsilon
        dotZ
    end

%% METHODS
    methods
    %% CONSTRUCTOR
    function obj = Drone(params, initStates, initInputs,simTime)
            obj.g = 9.81;
            obj.t = 0.0;
            obj.dt = 0.1;
            obj.tf = simTime;

            obj.m = params('mass');
            obj.b = params('gyro');
            obj.M = params('inertialMatrix');
            
            obj.x = initStates('position');
            obj.v = initStates('velocity');
            obj.R = initStates('attitude');
            euler = rotm2eul(obj.R);
            obj.phi = euler(3);
            obj.theta = euler(2);
            obj.psi = euler(1);
            obj.omega = initStates('angularV');
            
            obj.s = [obj.x; obj.v; obj.omega];
            obj.stateR = obj.R;
            obj.dotS = zeros(9,1);
            obj.dotR = zeros(3);

            obj.u = initInputs;
            obj.f = obj.u(1);
            obj.tau = obj.u(2:4);

            % error Control

            obj.x_err = 0.0;
            obj.x_err_prev = 0.0;
            obj.x_err_sum = 0.0;

            obj.v_err = 0.0;
            obj.v_err_prev = 0.0;
            obj.v_err_sum = 0.0;

            obj.dotV_err = 0.0;
            obj.dotV_err_prev = 0.0;
            obj.dotV_err_sum = 0.0;

            obj.R_des = zeros(3);
            obj.omega_des = zeros(3,1);

            % position controller

            obj.k = 4;
            obj.k_x = 0.1;
            obj.K_f = eye(3);

            % control law

            obj.psi_d = pi/4;
            obj.dot_psi_d = 0;
            obj.e_f = zeros(3,1);


            % attitude law 
            % Initialize control parameters

            

            obj.alpha_1 = 1;
            obj.alpha_2 = 0.01;
            obj.lambda_c = 1;
        
            obj.k_i = 0.1;
        
            obj.Lambda_i = 10 * eye(3);
            obj.A = 20; 
            obj.K_c = eye(3);
            obj.dot_bar_b = zeros(3,1);



            obj.r_1 = [0, 0, 1]';
            obj.r_2 = (1/sqrt(3))*[1, 1, 1]';
            obj.r_3 = cross(obj.r_1,obj.r_2)/ norm(cross(obj.r_1,obj.r_2));
            obj.r_i = [obj.r_1, obj.r_2, obj.r_3];

            obj.v_i = obj.R'*obj.r_i;



            % error vector alignment error
            
            obj.epsilon = 0;
            obj.z = zeros(3, 1); 
            obj.J = zeros(3);

           

            % gyro-bias observer
            obj.dot_v_f_i = 0;
            obj.v_f_i = obj.v_i;
            obj.gamma_f = 10000;

            %attitude controler

            obj.epsilon = 0;

            % Calcula v_d,i
            obj.v_d_i = obj.R_des'*obj.r_i;
            obj.dot_v_i = 0;
            obj.dot_v_d_i= 0;

            % Inicializar las variables del filtro y la matriz A
            obj.vartheta_1 = zeros(3,1);
            obj.vartheta_2 = zeros(3,1);

            obj.dot_vartheta_1 = zeros(3,1);
            obj.dot_vartheta_2 = zeros(3,1);
            
            obj.A = diag([20, 20, 20]);

            

        end
        
    %% RETURNS DRONE STATE
        function state = GetState(obj)
            state.s = obj.s;
            state.R = obj.R;

        end
        
    %% STATE SPACE (DIFFERENTIAL) EQUATIONS: INCOMPLETE!
        function obj = EvalEOM(obj)
          
            % Body to Worl Rotation Matrix
            wRb = obj.R';

            % Translational Motions
            obj.dotS(1:3) = obj.v;
            obj.dotS(4:6) = 1 / obj.m *([0; 0; -obj.g * obj.m] + wRb * [0; 0; obj.f]);

            
            % Angular velocity
            obj.dotR = obj.R * wedgeMap(obj.omega);

            % Angular Acceleration
            obj.dotS(7:9) = (obj.M) \ (cross(obj.omega,obj.M*obj.omega) + obj.tau);
           
        end

        function ControlStatmet(obj)

            e_z = [0, 0, 1]';

            obj.v_err = obj.eta - obj.k_x * obj.x_err - Tanh(obj.e_f);

            % CONTROL  STATEMENT

            obj.dotS(1:3) = obj.v_err;

            % obj.dotS(4:6) = 1 / obj.m * (-obj.m * obj.g * e_z - obj.m * obj.ddx_d ...
            %     + obj.f * obj.R_des * e_z + obj.f * (obj.R - obj.R_des) * e_z);

            obj.dotS(4:6) = (1/obj.m) *(-obj.m * (obj.k - obj.k_x) * obj.eta ...
                + obj.k*Tanh(obj.e_f) - obj.k_x^2*obj.x_err);

            % Angular velocity
            obj.dotR = obj.R * wedgeMap(obj.omega);

            % Angular Acceleration
            obj.dotS(7:9) = (obj.M) \ (cross(obj.omega,obj.M*obj.omega) + obj.tau);

         end

    %% PREDICT NEXT DRONE STATE
        function obj = UpdateState(obj)
            
            obj.t = obj.t + obj.dt;
            
            % Find(update) the next state of obj.X
            % obj.EvalEOM();
            obj.ControlStatmet();
            obj.s = obj.s + obj.dotS.*obj.dt;
            obj.stateR = obj.R + obj.dotR.*obj.dt;

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            obj.e_f = obj.e_f + obj.dotE_f * obj.dt;
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            obj.x = obj.s(1:3);
            obj.v = obj.s(4:6);
            
            obj.R = obj.stateR;
            obj.euler = rotm2eul(obj.R); 
            obj.phi = obj.euler(3);
            obj.theta = obj.euler(2); 
            obj.psi = obj.euler(1);

            obj.omega = obj.s(7:9);

            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %obj.v_f_i = obj.dot_v_f_i;
            %obj.epsilon = obj.dotEpsilon;
            %obj.z = obj.dotZ;
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

            obj.vartheta_1 = obj.vartheta_1 + obj.dot_vartheta_1 * obj.dt;
            obj.vartheta_2 = obj.vartheta_2 + obj.dot_vartheta_2 * obj.dt;
            


            %%%%%%%%%%%%%%%%%%%%% Measure %%%%%%%%%%%%%%%%%%%%%%%%%

%             obj.omega(1) = obj.omega(1) + 0.2;
%             obj.omega(2) = obj.omega(2) + 0.1;
%             obj.omega(3) = obj.omega(3) - 0.1;
            
        end
        
    %% CONTROLLER
    function obj = PositionCtrl(obj, pos_des)
             
             
              e_z = [0, 0, 1]';
              c_d = [cos(obj.psi_d), sin(obj.psi_d), 0]';

          


             %%%%%%%%%%%%%%%%%%%   Position   %%%%%%%%%%%%%%%%%%%%%%

             obj.x_des = pos_des;


             %%%%%%%%%%%%%%%%%%%   Velocity   %%%%%%%%%%%%%%%%%%%%%%

             % obj.v_des = gradient(obj.x_des);
             obj.v_des = [gradient(obj.x_des(1)), gradient(obj.x_des(2)), ...
                 gradient(obj.x_des(3))]';

             %%%%%%%%%%%%%%%%%%% Acceleration %%%%%%%%%%%%%%%%%%%%%%

             %obj.dotV_des = gradient(obj.v_des);

             obj.dotV_des = [gradient(obj.v_des(1)), gradient(obj.v_des(2)), ...
                 gradient(obj.v_des(3))]';


             obj.ddx_d = obj.dotV_des;


             % Error de posición y velocidad
             obj.x_err = obj.x - obj.x_des;
             obj.v_err = obj.v - obj.v_des;

            


             %  Positioning Controller  law

             obj.eta = obj.v_err + obj.k_x * obj.x_err + Tanh(obj.e_f);

             obj.dotE_f = Cosh(obj.e_f)^2 * (-obj.K_f * Tanh(obj.e_f) + obj.k_x^2 ...
                 * (1-(1/obj.m))* obj.x_err - obj.k * obj.eta);

             obj.T = obj.m * obj.g * e_z + obj.m * obj.ddx_d + (obj.k * eye(3) ...
                   + obj.m * (obj.k_x * eye(3) + obj.K_f)) * Tanh(obj.e_f);


             %**********************************************************%
             obj.f = norm(obj.T);
             %**********************************************************%


             % % Trayectoria de actitud deseada
             % c_3 = obj.T/norm(obj.T);
             % c_2 = cross(c_3,c_d)/norm(cross(c_3,c_d));
             % c_1 = cross(c_2,c_3);
             % obj.R_des = [c_1, c_2, c_3];
             % obj.dotR_des = gradient(obj.R_des);
             % obj.omega_des = veeMap(obj.R_des'*obj.dotR_des);
             % 
             % 
             % obj.v_d_i = obj.R_des'*obj.r_i;
             % 
             % % Vector aligment error
             % 
             % % Calcula epsilon(t) y z(t)
             % 
             % for i = 1:3  % Asumiendo que tienes 3 sensores
             %     obj.epsilon = obj.epsilon + obj.k_i * (1 - dot(obj.v_i(:,i), obj.v_d_i(:,i)));
             %     obj.z = obj.z + obj.k_i * cross(obj.v_i(:,i), obj.v_d_i(:,i));
             % end
             % 
             % % disp('********** Epsion y z ***************')
             % % disp(obj.epsilon);
             % % %disp(obj.z);
             % % disp('*************************************')
             % 
             % % Derivate epsilon(t) y z(t)
             % 
             % obj.dotEpsilon = obj.z' * (obj.omega - obj.omega_des);
             % obj.dotZ = obj.J * (obj.omega - obj.omega_des) + cross(obj.z,obj.omega_des);
             % 
             % % disp('********** Epsion y z  derivadas ***************')
             % % disp(obj.dotEpsilon);
             % % disp(obj.dotZ);
             % % disp('*************************************')
             % 
             % 
             % % Calculating J and implementing the equations
             % 
             % for i = 1:size(obj.v_d_i, 2)
             %     obj.J = obj.J + obj.k_i * wedgeMap(obj.v_d_i(:,i))' * wedgeMap(obj.v_d_i(:,i));
             % end
             % 
             % % disp('************* J *************')
             % % disp(obj.J)
             % % disp('*************************************')
             % 
             % 
             % 
             % %  gyro-bias observer
             % 
             % % Ecuación 1: \hat{b} = \dot{\bar{b}} - \sum ^n_{i =1} k_i(v^\wedge_{f,i})^T \Lambda_iv_i
             % 
             % sum_term = zeros(3,1);
             % 
             % for i = 1:3
             %     sum_term = sum_term + obj.k_i * wedgeMap(obj.v_f_i(:,i))' * obj.Lambda_i * obj.v_i(:,i);
             % end
             % 
             % obj.hat_b = obj.dot_bar_b - sum_term;
             % 
             % % disp('********** hat_b ***************')
             % % disp(obj.hat_b);
             % % disp('*************************************')
             % 
             % 
             % 
             % % angular velocity estimate
             % 
             % obj.omega_g = obj.omega + obj.b;
             % 
             % % disp('********** omega_g ***************')
             % % disp(obj.omega_g);
             % % disp('*************************************')
             % 
             % obj.hat_omega = obj.omega_g - obj.hat_b;
             % 
             % % disp('********** hat_omega_g ***************')
             % % disp(obj.hat_omega);
             % % disp('*************************************')
             % 
             % 
             % % Ecuación 2: \dot{\bar{b}} = K_f \hat{\omega} + \gamma_f \sum^n_{i=1} k_i (\Lambda_i v_i)^\wedge(v_i - v_{f,i})
             % sum_term = zeros(3,1);
             % 
             % for i = 1:3
             %     sum_term = sum_term + obj.k_i * cross((obj.Lambda_i * obj.v_i(:,i)),(obj.v_i(:,i) - obj.v_f_i(:,i)));
             % end
             % 
             % obj.dot_bar_b = obj.K_f * obj.hat_omega + obj.gamma_f * sum_term;
             % 
             % 
             % disp('********** dot_bar_b ***************')
             % disp(obj.dot_bar_b);
             % disp('*************************************')
             % 
             % 
             % obj.dot_v_f_i = obj.gamma_f * (obj.v_i - obj.v_f_i);
             % 
             % 
             % 
             % % for i = 1:3 % Asumiendo que tienes 3 sensores
             % %     obj.K_f = obj.K_f + obj.k_i * (wedgeMap(obj.v_f_i(:,i))' * obj.Lambda_i * wedgeMap(obj.v_i(:,i)));
             % % end
             % % obj.K_f = K_f; % Almacenar el resultado en la propiedad K_f del objeto
             % 
             % 
             % 
             % %  disp('********** dot_v_f_i ***************')
             % %  disp(obj.dot_v_f_i);
             % %  disp('*************************************')
             % %
             % %
             % obj.dotOmega_des = obj.vartheta_1;
             % 
             % 
             % % Calcular las derivadas del filtro
             % obj.dot_vartheta_1 = obj.vartheta_2;
             % obj.dot_vartheta_2 = -2*obj.A*obj.vartheta_2 - obj.A^2*(obj.vartheta_1 - obj.omega_des);
             % 
             % 
             % 
             % 
             % 
             % 
             % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
             % % Attitude controller law
             % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
             % 
             % obj.omega_r = - obj.lambda_c * obj.z + obj.omega_des;
             % 
             % obj.dot_hat_omega_r = - obj.lambda_c * obj.J * (obj.hat_omega - obj.omega_des) - obj.lambda_c * cross( obj.z, obj.omega_des) + obj.dotOmega_des;
             % 
             % 
             % obj.tau = obj.M * obj.dot_hat_omega_r - cross((obj.M * obj.hat_omega),obj.omega_r) - obj.K_c * (obj.hat_omega - obj.omega_r) - (obj.alpha_1*eye(3) + obj.alpha_2*obj.J')*obj.z;
             % 
             % %*******************************************************************************************************************************************************************************
             % 
             % 
             % 
             % 
             % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
             % % obj.u(1) = obj.m*obj.g;
             % % % obj.u(1) = 0.0;
             % % obj.u(2) = 0.0;   % roll(phi)    X
             % % obj.u(3) = 0.0;   % pith(theta)  Y
             % % obj.u(4) = 0.0;   % yaw(psi)     Z
             % %
             % %
             % %
             % % obj.f = obj.u(1);
             % % obj.tau = obj.u(2:4);
         end

         
    end
end
