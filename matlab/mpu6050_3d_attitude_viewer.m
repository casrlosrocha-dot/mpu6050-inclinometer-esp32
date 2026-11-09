clear; clc; close all;

% Ajusta al puerto COM de tu ESP32
puertoCOM = 'COM5'; 
baudRate = 115200;

% Limpiar conexiones previas bloqueadas en MATLAB
delete(findall(0, 'Type', 'matlab.peripherals.SerialPort'));

try
    s = serialport(puertoCOM, baudRate);
    configureTerminator(s, "LF");
    flush(s);
    disp("Conectado con éxito a la ESP32.");
catch ME
    error("No se pudo abrir el puerto serie. Revisa que el puerto sea COM5 y que el Monitor Serie de Arduino esté CERRADO.");
end

% Configuración de interfaz gráfica
figure('Name', 'Visualizador Tiempo Real MPU6050 - UMNG (Avión 3D)', 'NumberTitle', 'off', 'Color', [0.15 0.15 0.15]);

% Subplot 1: Aceleración
subplot(2,2,1);
hAccX = animatedline('Color','r','LineWidth',1.5);
hAccY = animatedline('Color','g','LineWidth',1.5);
hAccZ = animatedline('Color','b','LineWidth',1.5);
title('Aceleración (m/s^2)', 'Color', 'w'); legend('Ax','Ay','Az', 'TextColor', 'w'); grid on; xlabel('Muestras', 'Color', 'w');
set(gca, 'Color', [0.2 0.2 0.2], 'XColor', 'w', 'YColor', 'w');

% Subplot 2: Velocidad Angular
subplot(2,2,2);
hGyrX = animatedline('Color','r','LineWidth',1.5);
hGyrY = animatedline('Color','g','LineWidth',1.5);
hGyrZ = animatedline('Color','b','LineWidth',1.5);
title('Velocidad Angular (rad/s)', 'Color', 'w'); legend('Gx','Gy','Gz', 'TextColor', 'w'); grid on; xlabel('Muestras', 'Color', 'w');
set(gca, 'Color', [0.2 0.2 0.2], 'XColor', 'w', 'YColor', 'w');

% Subplot 3: Temperatura
subplot(2,2,3);
hTemp = animatedline('Color','m','LineWidth',1.5);
title('Temperatura (°C)', 'Color', 'w'); grid on; xlabel('Muestras', 'Color', 'w');
set(gca, 'Color', [0.2 0.2 0.2], 'XColor', 'w', 'YColor', 'w');

% Subplot 4: Modelo 3D Avión
subplot(2,2,4);
[V_orig, F, C] = airplane_geometry();
hAvion = patch('Faces', F, 'Vertices', V_orig, 'FaceVertexCData', C, 'FaceColor', 'flat', 'EdgeColor', 'k');
axis equal; grid on; xlabel('X', 'Color', 'w'); ylabel('Y', 'Color', 'w'); zlabel('Z', 'Color', 'w');
xlim([-2.5 2.5]); ylim([-2.5 2.5]); zlim([-2.5 2.5]);
view(3); title('Orientación 3D (Modelo Avión)', 'Color', 'w');
set(gca, 'Color', [0.2 0.2 0.2], 'XColor', 'w', 'YColor', 'w');

count = 0;
yaw = 0; % Inicialización de guiñada
dt = 0.05; % Período de muestreo (20 Hz = 50 ms)

disp("Visualización en tiempo real iniciada. Presiona Ctrl+C para detener.");

% Bucle de recepción y graficado en tiempo real
while ishandle(hAvion)
    if s.NumBytesAvailable > 0
        linea = readline(s);
        datos = str2double(split(linea, ','));
        
        if numel(datos) == 7 && ~any(isnan(datos))
            count = count + 1;
            ax = datos(1); ay = datos(2); az = datos(3);
            gx = datos(4); gy = datos(5); gz = datos(6);
            temp = datos(7);
            
            % Actualizar gráficas temporales
            addpoints(hAccX, count, ax);
            addpoints(hAccY, count, ay);
            addpoints(hAccZ, count, az);
            
            addpoints(hGyrX, count, gx);
            addpoints(hGyrY, count, gy);
            addpoints(hGyrZ, count, gz);
            
            addpoints(hTemp, count, temp);
            
            % Mover ventana gráfica (últimas 100 muestras)
            if count > 100
                subplot(2,2,1); xlim([count-100, count]);
                subplot(2,2,2); xlim([count-100, count]);
                subplot(2,2,3); xlim([count-100, count]);
            end
            
            % --- CÁLCULO DE ÁNGULOS DE ORIENTACIÓN ---
            % Roll (Alabeo) y Pitch (Cabeceo) a partir del Acelerómetro
            roll = atan2(ay, az);
            pitch = atan2(-ax, sqrt(ay^2 + az^2));
            
            % Integración básica del giro en Z para Yaw (Guiñada)
            if abs(gz) > 0.05 % Filtro de umbral de ruido
                yaw = yaw + gz * dt;
            end
            
            % Matriz de Rotación 3D (Euler Z-Y-X)
            Rz = [cos(yaw) -sin(yaw) 0; sin(yaw) cos(yaw) 0; 0 0 1];
            Ry = [cos(pitch) 0 sin(pitch); 0 1 0; -sin(pitch) 0 cos(pitch)];
            Rx = [1 0 0; 0 cos(roll) -sin(roll); 0 sin(roll) cos(roll)];
            
            R = Rz * Ry * Rx; % Matriz de rotación combinada
            
            % Aplicar rotación a los vértices del avión
            V_rot = (R * V_orig')';
            set(hAvion, 'Vertices', V_rot);
            drawnow limitrate;
        end
    end
end

% --- GEOMETRÍA DEL AVIÓN 3D ---
function [V, F, C] = airplane_geometry()
    % Vértices del avión [X, Y, Z]
    V = [
         2.0,  0.0,  0.0;  % 1: Nariz
        -1.5,  0.3,  0.3;  % 2: Fuselaje sup izq
        -1.5, -0.3,  0.3;  % 3: Fuselaje sup der
        -1.5, -0.3, -0.3;  % 4: Fuselaje inf der
        -1.5,  0.3, -0.3;  % 5: Fuselaje inf izq
         0.2,  2.2,  0.0;  % 6: Punta ala izquierda
        -0.5,  2.2,  0.0;  % 7: Borde salida ala izq
         0.2, -2.2,  0.0;  % 8: Punta ala derecha
        -0.5, -2.2,  0.0;  % 9: Borde salida ala der
        -1.5,  0.0,  1.0;  % 10: Punta timón vertical
        -1.8,  0.0,  1.0;  % 11: Atrás timón vertical
        -1.8,  0.0,  0.0   % 12: Cola posterior
    ];

    % Caras (Polígonos)
    F = [
        1 2 3 1;  % Fuselaje Superior
        1 3 4 1;  % Fuselaje Derecho
        1 4 5 1;  % Fuselaje Inferior
        1 5 2 1;  % Fuselaje Izquierdo
        2 6 7 5;  % Ala Izquierda
        3 8 9 4;  % Ala Derecha
        2 10 11 12 % Timón de Cola
    ];

    % Colores para cada sección (RGB)
    C = [
        0.8 0.2 0.2; % Rojo (Fuselaje)
        0.8 0.2 0.2;
        0.7 0.1 0.1;
        0.7 0.1 0.1;
        0.2 0.6 0.9; % Azul (Ala Izq)
        0.2 0.6 0.9; % Azul (Ala Der)
        0.9 0.8 0.1  % Amarillo (Timón)
    ];
end