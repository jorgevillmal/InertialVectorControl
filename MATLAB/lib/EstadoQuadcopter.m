classdef EstadoQuadcopter < handle
    % EstadoQuadcopter maneja la actualización del estado de un quadcopter.
    % Utiliza las ecuaciones de movimiento para calcular la dinámica del sistema.

    properties
        quadcopter           % Instancia de Quadcopter para acceder y modificar su estado.
        ecuacionesMovimiento % Instancia de EcuacionesDeMovimiento para calcular la dinámica.
    end

    methods
        function obj = EstadoQuadcopter(quadcopter, ecuacionesMovimiento)
            % Constructor del objeto EstadoQuadcopter.
            % Verifica que los argumentos necesarios se pasen al constructor.
            if nargin ~= 2
                error('EstadoQuadcopter requires a Quadcopter instance and EcuacionesDeMovimiento instance.');
            end
            obj.quadcopter = quadcopter;
            obj.ecuacionesMovimiento = ecuacionesMovimiento;
        end

        function updateState(obj, dt)
            % Actualiza el estado del quadcopter basándose en las derivadas del estado.
            % Argumentos:
            %   dt: Incremento de tiempo para la actualización del estado.
            
            % Obtener el estado actual del quadcopter.
            state = obj.quadcopter.getState();

            % Calcular las derivadas del estado usando las ecuaciones de movimiento.
            [dotPos, dotVel, dotOmega] = obj.ecuacionesMovimiento.evalEOM(state, ...
                obj.quadcopter.mass, obj.quadcopter.inertialMatrix);

            % Integrar las derivadas para obtener el nuevo estado.
            newState.position = state.position + dotPos * dt;
            newState.velocity = state.velocity + dotVel * dt;
            newState.angularVelocity = state.angularVelocity + dotOmega * dt;

            % Actualizar la orientación usando la matriz de rotación.
            R = state.orientation;
            omegaHat = obj.ecuacionesMovimiento.wedge(dotOmega);
            dR = R * omegaHat;
            newState.orientation = R + dR * dt;

            % Normalizar la nueva matriz de rotación para corregir errores numéricos.
            [U, ~, V] = svd(newState.orientation);
            newState.orientation = U * V';

            % Actualizar la propiedad time del quadcopter.
            obj.quadcopter.time = obj.quadcopter.time + dt;

            % Actualizar el estado del Quadcopter con los nuevos valores.
            obj.quadcopter.setState(newState);
        end

        function state = getState(obj)
            % Devuelve el estado actual del quadcopter.
            % Salida:
            %   state: Una estructura que contiene la posición, velocidad,
            %          orientación y velocidad angular actuales del quadcopter.
            
            state = obj.quadcopter.getState();
        end
    end
end