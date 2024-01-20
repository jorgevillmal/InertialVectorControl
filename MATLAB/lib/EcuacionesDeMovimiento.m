classdef EcuacionesDeMovimiento
    % EcuacionesDeMovimiento encapsula las ecuaciones que rigen el movimiento
    % de un cuadricóptero. Esta versión simplificada asume que no hay fuerzas
    % externas actuando sobre el cuadricóptero, y que se encuentra en lazo abierto.

    properties
        g = 0; % Aceleración gravitacional en m/s^2, se asume 0 para simulación sin gravedad.
    end

    methods
        function [dotPos, dotVel, dotOmega] = evalEOM(obj, state, m, M)
            % Evalúa las ecuaciones de movimiento del cuadricóptero.
            %
            % Argumentos:
            %   state: estructura con los siguientes campos:
            %       - velocity: vector de velocidad actual [vx, vy, vz].
            %       - orientation: matriz de rotación actual que representa la orientación.
            %       - angularVelocity: vector de velocidad angular [omega_x, omega_y, omega_z].
            %   m: masa del cuadricóptero.
            %   M: matriz de inercia del cuadricóptero.
            %
            % Devuelve:
            %   dotPos: derivada de la posición, que es la velocidad actual.
            %   dotVel: derivada de la velocidad, es decir, aceleración.
            %   dotOmega: derivada de la velocidad angular, es decir, aceleración angular.

            % Se asume que la fuerza de empuje y el torque son cero para un lazo abierto.
            f = 0; % Fuerza de empuje.
            tau = [0; 0; 0]; % Torque.

            % Ecuación translacional.
            gravity = [0; 0; -obj.g]; % Vector de gravedad.
            translationalAcc = gravity + f / m; % Aceleración translacional.

            % Ecuación rotacional.
            omegaHat = obj.wedge(state.angularVelocity); % Matriz antisimétrica de velocidad angular.
            dotR = state.orientation * omegaHat; % Derivada de la matriz de rotación.

            % Ecuación de momento angular.
            gyroEffect = cross(state.angularVelocity, (M * state.angularVelocity)); % Efecto giroscópico.
            angularAcc = M \ (gyroEffect + tau); % Aceleración angular.

            % Combinación de derivadas de estado.
            dotPos = state.velocity; % La derivada de la posición es la velocidad.
            dotVel = translationalAcc; % La derivada de la velocidad es la aceleración translacional.
            dotOmega = angularAcc; % La derivada de la velocidad angular es la aceleración angular.
        end

        function wedgeMatrix = wedge(~, omega)
            % Genera una matriz antisimétrica a partir de un vector de velocidad angular.
            %
            % Argumentos:
            %   omega: vector de velocidad angular [omega_x, omega_y, omega_z].
            %
            % Devuelve:
            %   wedgeMatrix: matriz antisimétrica correspondiente.

            wedgeMatrix = [ 0,       -omega(3),  omega(2);
                            omega(3), 0,        -omega(1);
                           -omega(2), omega(1), 0        ];
        end
    end
end
