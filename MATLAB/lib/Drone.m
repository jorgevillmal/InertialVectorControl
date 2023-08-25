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
    end

    properties
        % Attitude Controller
        alpha_1
        alpha_2
        lambda_c
        K_c

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


            % parameters
            obj.omega_g = obj.omega + obj.b;
            % attitude controller
            obj.alpha_1 = 1;
            obj.alpha_2 = 0.01;
            obj.lambda_c = 1;
            obj.K_c = eye(3);

            % Inertial Vector
            obj.r_1 = [0, 0, 1]';
            obj.r_2 = (1/sqrt(3))*[1, 1, 1]';
            obj.r_3 = cross(obj.r_1,obj.r_2)/ norm(cross(obj.r_1,obj.r_2));
            obj.r_i = [obj.r_1, obj.r_2, obj.r_3];

            obj.k_i = 0.1;

            obj.v_i = obj.R' * obj.r_i;
           


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

        end

        %% STATE SPACE (DIFFERENTIAL) EQUATIONS: INCOMPLETE!
        function obj = EvalEOM(obj)
            % EvalEOM: Evalúa las ecuaciones del movimiento del dron.

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
            % ControlStatmet: Determina las leyes de control para el dron.


            % CONTROL  STATEMENT

            % Translational Motions
            obj.dotS(1:3) = obj.eta - obj.k_x * obj.x_err - Tanh(obj.e_f);

            obj.dotS(4:6) = (1/obj.m) *(-obj.m * (obj.k - obj.k_x) * obj.eta ...
                + obj.k*Tanh(obj.e_f) - obj.k_x^2*obj.x_err);


            % Angular velocity
            obj.dotR = obj.R * wedgeMap(obj.omega);




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
            % obj.EvalEOM();
            obj.ControlStatmet();
            obj.s = obj.s + obj.dotS.*obj.dt;
            obj.stateR = obj.R + obj.dotR.*obj.dt;

            % Re-ortogonaliza la matriz de rotación para asegurar que sigue siendo una matriz ortogonal.
            % Esto ayuda a mitigar errores numéricos y asegura que R sigue siendo una matriz de rotación válida.
            [U, ~, V] = svd(obj.R);
            obj.R = U * V';

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
            % gyro-bias observer
            obj.dot_v_f_i = obj.gamma_f * (obj.v_i - obj.v_f_i);
            obj.v_f_i = obj.v_f_i + obj.dot_v_f_i * obj.dt;

            obj.dot_bar_b = obj.K_f * obj.hat_omega;
            for i = 1:size(obj.v_i, 2)
                obj.dot_bar_b = obj.dot_bar_b + obj.gamma_f * cross(obj.Lambda_i * obj.v_i(:,i), obj.v_i(:,i) - obj.v_f_i(:,i));
            end
            obj.bar_b = obj.bar_b + obj.dot_bar_b * obj.dt;
            % disp('obj.dot_bar_b')
            % disp(obj.dot_bar_b)


            % Filter dotOmega_des
            obj.dot_vartheta_1 = obj.vartheta_2;
            obj.dot_vartheta_2 = -2 * obj.A * obj.vartheta_2 - obj.A^2 * (obj.vartheta_1 - obj.omega_des);

            obj.dotOmega_des = obj.dot_vartheta_2;
            

            % Calculo de la matriz J
            for i = 1:size(obj.v_i, 2)
                obj.J = obj.J + obj.k_i * (wedgeMap(obj.v_d_i(:, i))' * wedgeMap(obj.v_i));
            end

            obj.dot_hat_omega_r = - obj.lambda_c * obj.J * (obj.hat_omega - obj.omega_des) - obj.lambda_c * ...
                cross(obj.z, obj.omega_des) + obj.dotOmega_des;
            obj.omega_r = obj.omega_r + obj.dot_hat_omega_r * obj.dt;

            % alignment error variables
            % Calculo de la tasa de cambio de epsilon
            obj.dotEpsilon = obj.z' * obj.omega_err - obj.lambda_c * obj.z' * obj.z;
            obj.epsilon = obj.epsilon + obj.dotEpsilon * obj.dt;

            % Cálculo de la tasa de cambio de z
            obj.dotZ = obj.J * obj.omega_err - obj.lambda_c * obj.J * obj.z + cross(obj.z, obj.omega_des);
            obj.z = obj.z + obj.dotZ * obj.dt;

            % bias estimation error
            obj.dot_b_err = - obj.K_f * obj.b_err;
            obj.b_err = obj.b_err + obj.dot_b_err * obj.dt;


        end

        %% CONTROLLER
        function obj = PositionCtrl(obj, pos_des, vel_des, acc_des)

            e_z = [0, 0, 1]';
            
            

           
           % Asignar a las variables internas
           obj.x_des = pos_des;
           obj.v_des = vel_des;
           obj.ddx_d = acc_des;

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



            % Trayectoria de actitud deseada

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

            % Para dotRd
            % Usando la relación: dotRd = Rd * omega_d^wedge
            % Donde omega_d^wedge = Rd' * dotRd y omega_d = (Rd' * dotRd)^vee

            % Como omega_d = (Rd' * dotRd)^vee, y dotRd = Rd * omega_d^wedge
            % Podemos reescribir omega_d = (Rd' * Rd * omega_d^wedge)^vee
            % Esto implica que omega_d es la derivada de los ángulos de Euler de Rd

            % Derivada de los ángulos de Euler
            euler_angles = rotm2eul(obj.R_des);


            dot_euler_angles = [0; 0; obj.dot_psi_d]; % Solo psi_d cambia


            % Convertir la derivada de los ángulos de Euler a velocidad angular
            obj.omega_des = eul2omega(dot_euler_angles, euler_angles);



            % obj.omega_des = veeMap(obj.R_des'*obj.dotR_des);

            obj.dotR_des = obj.R_des * wedgeMap(obj.omega_des);

            obj.v_d_i = obj.R_des' * obj.r_i;

            
            % Error angular y gyro bias
            obj.omega_err = obj.omega - obj.omega_r;
            obj.b_err = obj.hat_b - obj.b;

            % Vector alignment error

            obj.epsilon = 0;  % Reiniciar epsilon a cero
            obj.z = zeros(3,1);  % Reiniciar z a un vector cero


            for i = 1:size(obj.v_i, 2)
                obj.epsilon = obj.epsilon + obj.k_i * (1 - dot(obj.v_i(:,i)',obj.v_d_i(:,i)));
                obj.z = obj.z + obj.k_i * cross(obj.v_i(:,i), obj.v_d_i(:,i));
            end
 

            % gyro-bias observer
            obj.hat_b = obj.bar_b;
            for i = 1:size(obj.v_i, 2)
                obj.hat_b = obj.hat_b - obj.k_i * wedgeMap(obj.v_f_i(:,i))' * obj.Lambda_i * obj.v_i(:,i);
            end



            obj.hat_omega = obj.omega_g - obj.hat_b;

            


            obj.tau = obj.M * obj.dot_hat_omega_r - cross(obj.M * obj.hat_omega,obj.omega_r) - obj.K_c *(obj.hat_omega - obj.omega_r) ...
                - (obj.alpha_1 * eye(3) + obj.alpha_2 * obj.J')* obj.z;



            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % obj.u(1) = obj.m*obj.g;
            % % obj.u(1) = 0.0;
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
