classdef Simulador < handle
    % Simulador lleva a cabo la simulación de la dinámica de un quadcopter.

    properties
        quadcopter          % Objeto Quadcopter que almacena el estado del dron.
        estadoQuadcopter    % Objeto EstadoQuadcopter para actualizar el estado del dron.
        simulationTime      % Tiempo total de simulación en segundos.
        dt                  % Delta de tiempo para cada paso de simulación.
        currentTime         % Contador de tiempo actual dentro de la simulación.
    end

    methods
        function obj = Simulador(quadcopter, estadoQuadcopter, simulationTime, dt)
            % Constructor para el objeto Simulador.
            % Valida los parámetros de entrada y inicializa el simulador.
            if ~isa(quadcopter, 'Quadcopter')
                error('Simulador:InvalidParameter', 'El primer argumento debe ser una instancia de Quadcopter.');
            end
            if ~isa(estadoQuadcopter, 'EstadoQuadcopter')
                error('Simulador:InvalidParameter', 'El segundo argumento debe ser una instancia de EstadoQuadcopter.');
            end
            if ~isnumeric(simulationTime) || ~isnumeric(dt) || simulationTime <= 0 || dt <= 0
                error('Simulador:InvalidParameter', 'El tiempo de simulación y dt deben ser numéricos y mayores que cero.');
            end
            obj.quadcopter = quadcopter;
            obj.estadoQuadcopter = estadoQuadcopter;
            obj.simulationTime = simulationTime;
            obj.dt = dt;
            obj.currentTime = 0;
        end

        function start(obj)
            % Ejecuta la simulación completa basada en el tiempo de simulación y dt.
            while obj.currentTime < obj.simulationTime
                obj.estadoQuadcopter.updateState(obj.dt);
                obj.quadcopter.time = obj.currentTime;
                obj.currentTime = obj.currentTime + obj.dt;
                if obj.currentTime > obj.simulationTime
                    obj.currentTime = obj.simulationTime;
                    obj.quadcopter.time = obj.currentTime;
                    break; % Finaliza la simulación si se alcanza el tiempo total.
                end
            end
        end

        function step(obj)
            % Ejecuta un único paso de simulación y actualiza el estado del quadcopter.
            obj.estadoQuadcopter.updateState(obj.dt);
            obj.currentTime = obj.currentTime + obj.dt;
            % Asegura que el tiempo del quadcopter esté sincronizado con el tiempo de simulación.
            assert(obj.quadcopter.time == obj.currentTime, 'Desincronización del tiempo entre Quadcopter y Simulador.');
        end
    end
end