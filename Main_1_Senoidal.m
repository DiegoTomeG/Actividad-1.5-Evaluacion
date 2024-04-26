clear
close all
clc

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% TIEMPO %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

tf = 7;             % Tiempo de simulación en segundos (s)
ts = 0.05;           % Tiempo de muestreo en segundos (s)
t = 0: ts: tf;      % Vector de tiempo
N = length(t);      % Muestras

%%%%%%%%%%%%%%%%%%%%%%%%% DEFINICION DE LA FUNCIÓN %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Definir el rango de x
a = 5;
x_range = linspace(0, a, N);

% Calcular los valores de la función senoidal 
F_x = 2 * sin(x_range.^1.5);

% Calcular la derivada
dF_x = gradient(F_x, x_range);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%% BUCLE DE SIMULACION %%%%%%%%%%%%%%%%%%%%%%%%%%%%

x1 = zeros(1, N+1);    % Posición en el centro del eje que une las ruedas (eje x) en metros (m)
y1 = zeros(1, N+1);    % Posición en el centro del eje que une las ruedas (eje y) en metros (m)
phi = zeros(1, N+1);   % Orientación del robot en radianes (rad)

x1(1) = 0;    % Posición inicial eje x
y1(1) = 0;    % Posición inicial eje y
phi(1) = pi/4; % Orientación inicial del robot

for k = 1:N-1
    
    % Calculamos la orientación del robot basado en función senoidal en el punto actual
    phi(k+1) = atan(dF_x(k));
    
    % Calculamos el cambio en x,y
    delta_x = ts * cos(phi(k+1));
    delta_y = ts * sin(phi(k+1));
    
    % Calculamos las nuevas posiciones del robot
    x1(k+1) = x1(k) + delta_x;
   
    % Ajustamos la posición y para que siga la forma de la gráfica
    y1(k+1) = F_x(k+1);
    
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%% SIMULACION VIRTUAL 3D %%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Configuración de la figura
scene = figure;
set(scene,'Color','white');
set(gca,'FontWeight','bold');
sizeScreen = get(0,'ScreenSize');
set(scene,'position',sizeScreen);
camlight('headlight');
axis equal;
grid on;
xlabel('x(m)'); ylabel('y(m)'); zlabel('z(m)');
view([135 35]);
axis([-10 10 -10 10 0 4]);

% Graficar robot en la posición inicial
scale = 4;
MobileRobot_5;
H1 = MobilePlot_4(x1(1), y1(1), phi(1), scale);
hold on;

% Graficar trayectoria
H2 = plot3(x1, y1, zeros(size(x1)), 'g', 'LineWidth', 2);

% Bucle de simulación de movimiento del robot
step = 1;

for k = 1:step:N

    delete(H1);
    delete(H2);
    
    H1 = MobilePlot_4(x1(k), y1(k), phi(k), scale);
    H2 = plot3(x1(1:k), y1(1:k), zeros(1, k), 'g', 'LineWidth', 2);
    
    pause(ts);

end

% Graficar la trayectoria final
graph = figure;
set(graph,'position',sizeScreen);
plot3(x1, y1, zeros(size(x1)), 'r', 'LineWidth', 2);
xlabel('x(m)'); ylabel('y(m)'); zlabel('z(m)');
title('Trayectoria del robot');
grid on;
axis equal;



