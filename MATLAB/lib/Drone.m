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
        bar_b


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


        % State
        s
        stateR
        dotS
        dotR

        % Input
        u
        f
        tau
    end

    %% Control Problem

    properties
        % Position-tracking

        x_des       % desired position
        x_err       % error position
        
        v_des       % desired velocity
        v_err       % error position
        

        dotV_des    % desired acceleration
        dotV_err    % error acceleration

        prev_x_d
        prev_dx_d

        next_x_d
        next_dx_d

        dot_x_err
        dot_eta
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
        Tanh_ef
        dot_Tanh_ef
    end

    %% Attitude Problem
    properties
        omega_des   % desired omega
        omega_r
        omega_err
        b_err
        dot_hat_omega_r

        epsilon
        z

        % desired
        v_d_i

        % angular velocity estimate
        hat_omega
        hat_b



        % alignment error variable
        J

        dotEpsilon
        dotZ
       
        

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        R_des % desired attitude
        dotR_des 
        R_tilde
    end

    properties
        % Attitude Controller
        alpha_1
        alpha_2
        lambda_c
        K_c
        R_err

        % Bias Observer
        Lambda_i
        gamma_f

        v_f_i
        dot_v_f_i
        dot_bar_b
        dot_b_err

        % Filter
        A

        vartheta_1
        vartheta_2
        dot_vartheta_1
        dot_vartheta_2

        dotOmega_des

        % Inertial Vector
        r_1
        r_2
        r_3
        r_i
        k_i
    end


    %% METHODS
    methods
        % CONSTRUCTOR
        function obj = Drone(params, initStates, initInputs,simTime)
            e_z = [0, 0, 1]';

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
            obj.dotR_des = zeros(3);

            obj.u = initInputs;
            obj.f = obj.u(1);
            obj.tau = obj.u(2:4);

            % position controller
            obj.k = 4;
            obj.k_x = 0.1;
            obj.K_f = eye(3);

            % control law
            obj.psi_d = pi/4;
            obj.dot_psi_d = 0;
            obj.e_f = zeros(3,1);
            obj.x_err = zeros(3,1);
            obj.v_err = zeros(3,1);
            obj.ddx_d = zeros(3,1);
            
            obj.eta = obj.v_err + obj.k_x * obj.x_err + Tanh(obj.e_f);
            obj.T = obj.m * obj.g * e_z + obj.m * obj.ddx_d + (obj.k * eye(3) ...
                + obj.m * (obj.k_x * eye(3) + obj.K_f)) * Tanh(obj.e_f);
            obj.f = norm(obj.T);

            obj.Tanh_ef = Tanh(obj.e_f);
            obj.dot_Tanh_ef = zeros(3,1);
            obj.dot_x_err = zeros(3,1);
            obj.dot_eta = zeros(3,1);
            obj.dotE_f = zeros(3,1);


            % parameters
            obj.omega_g = obj.omega + obj.b;
            % attitude controller
            obj.alpha_1 = 1;
            obj.alpha_2 = 0.01;
            obj.lambda_c = 1;
            obj.K_c = eye(3);
            obj.omega_err = zeros(3,1);

            % Inertial Vector
            obj.r_1 = [0, 0, 1]';
            obj.r_2 = (1/sqrt(3))*[1, 1, 1]';
            obj.r_3 = cross(obj.r_1,obj.r_2)/ norm(cross(obj.r_1,obj.r_2));
            obj.r_i = [obj.r_1, obj.r_2, obj.r_3];

            obj.k_i = 0.1;

            obj.v_i = obj.R' * obj.r_i;
            obj.R_err = eye(3);

            
           


            % vector aligment error
            obj.epsilon = 0;
            obj.z = zeros(3,1);

            obj.J = zeros(3);

            obj.dotEpsilon = 0;
            obj.dotZ = zeros(3,1);



            % gyro-bias observer
            
            obj.b_err = zeros(3,1);
            obj.dot_b_err = zeros(3,1);
            obj.hat_b = zeros(3,1);
            obj.v_f_i = obj.v_i;
            obj.dot_b_err = - obj.K_f * obj.b_err;
            obj.Lambda_i = 10*eye(3);
            obj.gamma_f = 10000;

            obj.dot_bar_b = zeros(3,1);
            obj.hat_omega = zeros(3,1);

            % attitude law
            obj.bar_b = zeros(3,1);
            obj.omega_des = zeros(3,1);
            obj.omega_r = zeros(3,1);
            obj.omega_r = - obj.lambda_c * obj.z + obj.omega_des;
            obj.dot_hat_omega_r = zeros(3,1);

            % Inicializar las variables del filtro y la matriz A
            obj.vartheta_1 = zeros(3,1);
            obj.vartheta_2 = zeros(3,1);

            obj.dot_vartheta_1 = zeros(3,1);
            obj.dot_vartheta_2 = zeros(3,1);

            obj.dotOmega_des = zeros(3,1);

            % Filter
            obj.A = 20;

            % prueba de derivada x_d
            obj.x_des = zeros(3,1);
            obj.v_des = zeros(3,1);

            obj.prev_x_d = obj.x_des;
            obj.prev_dx_d = zeros(3,1);

            obj.next_x_d = zeros(3,1); % o cualquier valor inicial predeterminado
            obj.next_dx_d = zeros(3,1);
        end

        %% RETURNS DRONE STATE
        function state = GetState(obj)
            state.s = obj.s;
            state.R = obj.R;
            state.eta = norm(obj.eta);
            state.tanhe_f = norm(Tanh(obj.e_f));
            state.f = norm(obj.T);
            state.tau = norm(obj.tau);
            state.tilde_x = norm(obj.x_err);
        end

        %% STATE SPACE (DIFFERENTIAL) EQUATIONS: INCOMPLETE!
        function obj = EvalEOM(obj)
            % EvalEOM: Evalúa las ecuaciones del movimiento del dron.

            % Body to Worl Rotation Matrix
            %wRb = obj.R';

            % Translational Motions
            obj.dotS(1:3) = obj.eta - obj.k_x * obj.x_err - Tanh(obj.e_f);

            obj.dotS(4:6) = (1/obj.m) *(-obj.m * (obj.k - obj.k_x) * obj.eta ...
                + obj.k* Tanh(obj.e_f) - obj.k_x^2*obj.x_err);

            % Angular velocity
            obj.dotR = obj.R_des * wedgeMap(obj.omega_des);
            

            G = obj.K_c - wedgeMap(obj.omega_r) * obj.M + obj.lambda_c * obj.M * obj.J;

            % Angular Acceleration
            obj.dotS(7:9) = obj.M \ (cross(obj.M * obj.omega, obj.omega_err) - ...
                obj.K_c * obj.omega_err + G * obj.b_err - ...
                (obj.alpha_1 * eye(3) + obj.alpha_2 * obj.J')* obj.z);


        end

       

        %% PREDICT NEXT DRONE STATE
        function obj = UpdateState(obj)
            % UpdateState: Actualiza el estado del dron usando integración numérica.


            obj.t = obj.t + obj.dt;

            % Find(update) the next state of obj.X
            obj.EvalEOM();
            %obj.ControlStatmet();
            obj.s = obj.s + obj.dotS.*obj.dt;
            obj.stateR = obj.R + obj.dotR.*obj.dt;

          

            % Re-ortogonaliza la matriz de rotación para asegurar que sigue siendo una matriz ortogonal.
            % Esto ayuda a mitigar errores numéricos y asegura que R sigue siendo una matriz de rotación válida.
            [U, ~, V] = svd(obj.R);
            obj.R = U * V';

           
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
            obj.e_f = obj.e_f + obj.dotE_f * obj.dt;
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            

            % Calculo de la matriz J
            for i = 1:size(obj.v_i, 2)
                obj.J = obj.J + obj.k_i * (wedgeMap(obj.v_d_i(:, i))' * wedgeMap(obj.v_i));
            end

            % alignment error variables
            % Calculo de la tasa de cambio de epsilon
            obj.dotEpsilon = obj.z' * obj.omega_err - obj.lambda_c * obj.z' * obj.z;
            obj.epsilon = obj.epsilon + obj.dotEpsilon * obj.dt;

            % Cálculo de la tasa de cambio de z
            obj.dotZ = obj.J * obj.omega_err - obj.lambda_c * obj.J * obj.z + cross(obj.z, obj.omega_des);
            obj.z = obj.z + obj.dotZ * obj.dt;

            % gyro-bias observer
            obj.dot_bar_b = obj.K_f * obj.hat_omega;
            for i = 1:size(obj.v_i, 2)
                obj.dot_bar_b = obj.dot_bar_b + obj.gamma_f * cross(obj.Lambda_i * obj.v_i(:,i), obj.v_i(:,i) - obj.v_f_i(:,i));
            end
            obj.bar_b = obj.bar_b + obj.dot_bar_b * obj.dt;
        

            obj.dot_v_f_i = obj.gamma_f * (obj.v_i - obj.v_f_i);
            obj.v_f_i = obj.v_f_i + obj.dot_v_f_i * obj.dt;

            obj.dot_b_err = - obj.K_f * obj.b_err;
            obj.b_err = obj.b_err + obj.dot_b_err * obj.dt;
           

        end

         function ControlStatmet(obj, pos_des, vel_des, acc_des)
             % ControlStatmet: Determina las leyes de control para el dron.


             % Posicion deseada
             obj.x_des = pos_des;
             obj.v_des = vel_des;
             obj.ddx_d = acc_des;

             


             % Error de posición y velocidad
             obj.x_err = obj.x - obj.x_des;
             obj.v_err = obj.v - obj.v_des;

             %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
             obj.psi_d = atan2(obj.x_des(2), obj.x_des(1));

             dx_dt = obj.v_des(1);
             dy_dt = obj.v_des(2);

             partial_psi_d_x = -obj.x_des(2) / (obj.x_des(1)^2 + obj.x_des(2)^2);
             partial_psi_d_y = obj.x_des(1) / (obj.x_des(1)^2 + obj.x_des(2)^2);

             obj.dot_psi_d = partial_psi_d_x * dx_dt + partial_psi_d_y * dy_dt;


             c_d = [cos(obj.psi_d); sin(obj.psi_d); 0];


             c_3 = obj.T/norm(obj.T);
             c_2 = cross(c_3,c_d)/norm(cross(c_3,c_d));
             c_1 = cross(c_2,c_3);

             obj.R_des = [c_1, c_2, c_3];

             obj.R_err = obj.R * obj.R';
             obj.b_err = obj.hat_b - obj.b;
             obj.omega_err = obj.omega - obj.omega_r;

             obj.v_i = obj.R' * obj.r_i;
             obj.v_d_i = obj.R_des' * obj.r_i;

             % gyro-bias observer
             obj.hat_b = obj.bar_b;
             for i = 1:size(obj.v_i, 2)
                 obj.hat_b = obj.hat_b - obj.k_i * wedgeMap(obj.v_f_i(:,i))' * obj.Lambda_i * obj.v_i(:,i);
             end

             % Vector alignment error

             for i = 1:size(obj.v_i, 2)
                 obj.epsilon = obj.epsilon + obj.k_i * (1 - dot(obj.v_i(:,i)',obj.v_d_i(:,i)));
                 obj.z = obj.z + obj.k_i * cross(obj.v_i(:,i), obj.v_d_i(:,i));
             end

             

             for i = 1:size(obj.v_i, 2)
                 obj.K_f = obj.k_i * wedgeMap(obj.v_f_i(:,i))' * obj.Lambda_i * wedgeMap(obj.v_i(:,i));
             end




             obj.PositionCtrl();
             obj.AttitudeCtrl();
        end

        %% CONTROLLER
        function obj = PositionCtrl(obj)
            e_z = [0, 0, 1]';
            
            obj.eta = obj.v_err + obj.k_x * obj.x_err + Tanh(obj.e_f);

            % Usar la función para obtener el vector Cosh^2(ef)
            squared_values = CoshSquaredVector(obj.e_f);

            % Calcular la siguiente parte de la ecuación
            vector_part = (-obj.K_f * Tanh(obj.e_f) + obj.k_x^2 * (1-(1/obj.m))* obj.x_err - obj.k * obj.eta);

            % Hacer el producto escalar
            obj.dotE_f = squared_values * vector_part;

            obj.T = obj.m * obj.g * e_z + obj.m * obj.ddx_d + (obj.k * eye(3) ...
                + obj.m * (obj.k_x * eye(3) + obj.K_f)) * Tanh(obj.e_f);

            %**********************************************************%
            obj.f = norm(obj.T);
            %**********************************************************%






            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % obj.u(1) = obj.m*obj.g;
            % obj.u(1) = 0.0;
            % obj.u(2) = 0.0;   % roll(phi)    X
            % obj.u(3) = 0.0;   % pith(theta)  Y
            % obj.u(4) = 0.0;   % yaw(psi)     Z
            %
            %
            %
            % obj.f = obj.u(1);
            % obj.tau = obj.u(2:4);
        end

        function obj = AttitudeCtrl(obj)

            obj.omega_r = - obj.lambda_c * obj.z + obj.omega_des;

            obj.dot_hat_omega_r = - obj.lambda_c * obj.J * (obj.hat_omega - obj.omega_des) - obj.lambda_c * ...
                cross(obj.z, obj.omega_des) + obj.dotOmega_des;

            obj.tau = obj.M * obj.dot_hat_omega_r - cross(obj.M * obj.hat_omega,obj.omega_r) - obj.K_c *(obj.hat_omega - obj.omega_r) ...
                - (obj.alpha_1 * eye(3) + obj.alpha_2 * obj.J')* obj.z;



            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % obj.u(1) = obj.m*obj.g;
            % obj.u(1) = 0.0;
            % obj.u(2) = 0.0;   % roll(phi)    X
            % obj.u(3) = 0.0;   % pith(theta)  Y
            % obj.u(4) = 0.0;   % yaw(psi)     Z
            %
            %
            %
            % obj.f = obj.u(1);
            % obj.tau = obj.u(2:4);
        end


    end
end
