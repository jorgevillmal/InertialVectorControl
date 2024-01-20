classdef Quadcopter < handle
    % Quadcopter define un modelo para un dron cuadricóptero en MATLAB.
    % Encapsula el estado del cuadricóptero, incluyendo su posición,
    % velocidad, orientación y otros parámetros físicos relevantes.

    properties
        position        % Vector [x, y, z] de posición en metros.
        velocity        % Vector [vx, vy, vz] de velocidad en m/s.
        orientation     % Matriz de rotación 3x3 que representa la orientación del cuadricóptero.
        angularVelocity % Vector [omega_x, omega_y, omega_z] de velocidad angular en rad/s.
        mass            % Masa del cuadricóptero en kilogramos.
        inertialMatrix  % Matriz de inercia 3x3 del cuadricóptero.
        gyro            % Vector [gyro_x, gyro_y, gyro_z] de la velocidad angular del giroscopio.
        controlInputs   % Estructura con entradas de control: roll, pitch, yaw, thrust.
        time            % Tiempo actual en la simulación en segundos.
    end

    methods
        function obj = Quadcopter(params, initStates, initInputs)
            % Constructor de la clase Quadcopter que inicializa un nuevo cuadricóptero.
            % params: Map con los parámetros físicos ('mass', 'gyro', 'inertialMatrix').
            % initStates: Map con los estados iniciales ('position', 'velocity', 'attitude', 'angularV').
            % initInputs: Estructura con las entradas iniciales de control.
            
            % Asignación directa de parámetros y estados iniciales a las propiedades del objeto.
            obj.mass = params('mass');
            obj.inertialMatrix = params('inertialMatrix');
            obj.gyro = params('gyro');
            obj.position = initStates('position');
            obj.velocity = initStates('velocity');
            obj.orientation = initStates('attitude');
            obj.angularVelocity = initStates('angularV');
            obj.controlInputs = initInputs;
            obj.time = 0; % Inicialización del tiempo de simulación a 0.
        end

        function setState(obj, newState)
            % Actualiza el estado del cuadricóptero con un nuevo conjunto de valores.
            % newState: Estructura con los nuevos valores de estado ('position', 'velocity', 'orientation', 'angularVelocity').
            
            % Actualización de las propiedades del objeto con los nuevos valores de estado.
            obj.position = newState.position;
            obj.velocity = newState.velocity;
            obj.orientation = newState.orientation;
            obj.angularVelocity = newState.angularVelocity;
            
            % Opcional: Actualizar el tiempo si se incluye en el nuevo estado.
            if isfield(newState, 'time')
                obj.time = newState.time;
            end
        end

        function state = getState(obj)
            % Devuelve el estado actual del cuadricóptero.
            % Retorna una estructura con la posición actual, la velocidad, la orientación y la velocidad angular.
            
            % Creación de una estructura de estado a partir de las propiedades actuales del objeto.
            state.position = obj.position;
            state.velocity = obj.velocity;
            state.orientation = obj.orientation;
            state.angularVelocity = obj.angularVelocity;
            state.time = obj.time;
        end

        function [mass, gyro, inertialMatrix] = getPhysicalProperties(obj)
            % Devuelve las propiedades físicas del cuadricóptero.
            % Retorna la masa, el vector gyro y la matriz de inercia.
            
            % Asignación directa de las propiedades físicas a las variables de salida.
            mass = obj.mass;
            gyro = obj.gyro;
            inertialMatrix = obj.inertialMatrix;
        end
    end
end
