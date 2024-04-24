clear
close all
clc

%%% TIEMPO %%%
 
% Definición de parámetros de tiempo
tf = 14;             % Tiempo de simulación en segundos (s)
ts = 1;              % Tiempo de muestreo en segundos (s)

% Creación del vector de tiempo
t = 0:ts:tf;         % Vector de tiempo
N = length(t);       % Número de muestras

%%% CONDICIONES INICIALES %%%

x1 = zeros (1,N+1);  % Posición en el centro del eje que une las ruedas (eje x) en metros (m)
y1 = zeros (1,N+1);  % Posición en el centro del eje que une las ruedas (eje y) en metros (m)
phi = zeros(1, N+1); % Orientacion del robot en radianes (rad)

x1(1) = 0;    % Posición inicial eje x
y1(1) = 0;   % Posición inicial eje y
phi(1) = 0;   % Orientación inicial del robot

%%% PUNTO DE CONTROL %%%

hx = zeros(1, N+1);  % Posición en el punto de control (eje x) en metros (m)
hy = zeros(1, N+1);  % Posición en el punto de control (eje y) en metros (m)

hx(1) = x1(1); % Posición en el punto de control del robot en el eje x
hy(1) = y1(1); % Posición en el punto de control del robot en el eje y

%%% VELOCIDADES DE REFERENCIA %%% 
u = [ 0  , 2.33,       0 ,  4.60,       0,      4.12,      0 ,    4.12,     0,   4.12,       0,  4.12,      0, 4.12,      0, 0]; % Velocidad lineal de referencia (m/s)
w = [pi/3,  0  ,  4*pi/3 ,  0   ,  3*pi/4,         0,  7*pi/6,       0, 3*pi/4,     0,  7*pi/6,     0, 3*pi/4,    0, 7*pi/6,   0]; % Velocidad angular de referencia (rad/s)

%%% BUCLE DE SIMULACIÓN %%% 
for k=1:N 
    
    phi(k+1)=phi(k)+w(k)*ts; % Integral numérica (método de Euler)
    
    % MODELO CINEMATICO 
    xp1=u(k)*cos(phi(k+1)); 
    yp1=u(k)*sin(phi(k+1));

    x1(k+1)=x1(k) + xp1*ts ; % Integral numérica (método de Euler)
    y1(k+1)=y1(k) + yp1*ts ; % Integral numérica (método de Euler)
    
    % Posición del robot con respecto al punto de control
    hx(k+1)=x1(k+1); 
    hy(k+1)=y1(k+1);

end

% SIMULACION VIRTUAL 3D

% a) Configuración de escena
scene=figure;  % Crear figura (Escena)
set(scene,'Color','white'); % Color del fondo de la escena
set(gca,'FontWeight','bold') ;% Negrilla en los ejes y etiquetas
sizeScreen=get(0,'ScreenSize'); % Retorna el tamaño de la pantalla del computador
set(scene,'position',sizeScreen); % Congigurar tamaño de la figura
camlight('headlight'); % Luz para la escena
axis equal; % Establece la relación de aspecto para que las unidades de datos sean las mismas en todas las direcciones.
grid on; % Mostrar líneas de cuadrícula en los ejes
box on; % Mostrar contorno de ejes
xlabel('x(m)'); ylabel('y(m)'); zlabel('z(m)'); % Etiqueta de los eje

view([135 35]); % Orientacion de la figura
axis([-15 15 -15 15 0 4]); % Ingresar limites minimos y maximos en los ejes x y z [minX maxX minY maxY minZ maxZ]

% b) Graficar robots en la posicion inicial
scale = 4;
MobileRobot_5;
H1=MobilePlot_4(x1(1),y1(1),phi(1),scale);hold on;

% c) Graficar Trayectorias
H2=plot3(hx(1),hy(1),0,'r','lineWidth',2);

% d) Bucle de simulacion de movimiento del robot

step=1; % pasos para simulación

for k=1:step:N

    delete(H1);    
    delete(H2);
    
    H1=MobilePlot_4(x1(k),y1(k),phi(k),scale);
    H2=plot3(hx(1:k),hy(1:k),zeros(1,k),'r','lineWidth',2);
    
    pause(ts);

end