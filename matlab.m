% Programacion para graficar y analizar fuerzas, distancias y torques del
% dispositivo de rehabilitacion de tobillo
% requiere la lectura previa del informe de PFG

% Definir variables
r1 = 181.17;  %distancia deel eje de rotacion al centro del soporte de la banda
p2 = [166, 74, -336]; %ubicacion de la banda en los soportes inferiores
p22 = [166, -74, -336]; %punto para graficar
p0 = [0, 0, 0]; %centro de rotacion

% Definir los valores de angle2 (PL-DO)
angle2_min = -29;  % valor minimo de angle2
angle2_max = 15;   % valor maximo de angle2
angle2 = linspace(angle2_min, angle2_max, 100);  % 100 puntos entre -29 y 15

% Calcular theta
theta = deg2rad(angle2 - 39.4);  % Convertir de grados a radianes

%P1 neutro (no se necesita, solo para confirmar calculos)
p3x = r1 * cos(deg2rad(-39.4));
p3z = r1 * sin(deg2rad(-39.4));  
p3y = 0;

% Calcular las coordenadas del punto p1
p1x = r1 * cos(theta);  % Coordenada x de p1
p1z = r1 * sin(theta);  % Coordenada z de p1 (antes era y)
p1y = zeros(size(theta));  % Coordenada y de p1 es 0 ahora

% Calcular la distancia entre p1 y p2
distancia = sqrt((p1x - p2(1)).^2 + (p1y - p2(2)).^2 + (p1z - p2(3)).^2);

% Graficar la distancia en funcion de theta
figure;
plot(rad2deg(theta), distancia, 'LineWidth', 2);
xlabel('alfa (grados)');
ylabel('Distancia entre p1 y p2');
title('Distancia entre p1 y p2 en funcin de alfa');
grid on;

%graficar como se comporta p1x
%figure;
%plot(rad2deg(theta), p1x, 'LineWidth', 2);
%xlabel('alfa (grados)');
%ylabel('p1x');
%title('p1x en funcin de alfa');
%grid on;

% Graficar la trayectoria de p1 y la posicion de p2 en 3D
figure;
hold on;
plot3(p1x, p1y, p1z, 'b-', 'LineWidth', 2);  % Trayectoria de p1
line([0, p3x],[0, p3y],[0,p3z], 'Color', 'g', 'LineWidth', 1);
line([p3x, p22(1)],[p3y, p22(2)],[p3z, p22(3)], 'Color', '#D95319', 'LineWidth', 1);

plot3(p0(1), p0(2), p0(3), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'k');  % Punto p0
plot3(p3x, p3y, p3z, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'c');  % Punto p1
plot3(p22(1), p22(2), p22(3), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');  % Punto p2
line([-80, -80], [0, 0], [0, -115], 'Color', 'm', 'LineWidth', 2); 
line([-80, 140], [0, 0], [-115, -115], 'Color', 'm', 'LineWidth', 2);
%Soportes


% Limitar los ejes para ajustar el zoom
xlim([-100, 180]);  % Ajusta estos valores segn tu necesidad
ylim([-80, 20]);    % Ajusta el rango del eje Y
zlim([-370, 20]);   % Ajusta el rango del eje Z

xlabel('X');
ylabel('Y');
zlabel('Z');
title('Ubicacin de puntos y trayectoria de P1');
grid on;
axis equal;
legend('Trayectoria de P1','r1','r2', 'P0','P1', 'P2', 'Soportes pie');
view(45, 30);  % Cambiar los valores para ajustar la vista si es necesario
hold off;


% Definir variables
k = 2;  % 2 porciones de banda
distancia_inicial = 74;  % Distancia inicial de banda

% Calcular delta distancia
delta_distancia = distancia - distancia_inicial;

% Calcular la fuerza de cada banda segun las ecuaciones que las rigen
F_resorte =k*(-0.0003 * delta_distancia.^2 + 0.167 * delta_distancia+1.0758); 
F_verde =k*(-0.00007 * delta_distancia.^2 + 0.0832 * delta_distancia+0.1277); 
F_amarilla =k*(-0.0001 * delta_distancia.^2 + 0.1204 * delta_distancia+0.665); 
F_azul =k*(-0.0001 * delta_distancia.^2 + 0.1104 * delta_distancia + 0.266);
F_negra =k*(-0.0004 * delta_distancia.^2 + 0.2591 * delta_distancia+0.2591);

%graficar la deformacion en funcion del angulo
%figure;
%plot(rad2deg(theta), delta_distancia, 'LineWidth', 2);
%xlabel('alfa (grados)');
%ylabel('deltad');
%title('deltad en funcin de alfa');
%grid on;


% Calcular el vector unitario entre p2 y p1
vec_p2_p1_x =  p1x - p2(1);  % Diferencia en x
vec_p2_p1_y =  p1y - p2(2);  % Diferencia en y
vec_p2_p1_z =  p1z - p2(3);  % Diferencia en z

% Magnitud del vector entre p2 y p1
magnitud_p2_p1 = sqrt(vec_p2_p1_x.^2 + vec_p2_p1_y.^2 + vec_p2_p1_z.^2);

% Calcular el vector unitario
unitario_p2_p1_x = vec_p2_p1_x ./ magnitud_p2_p1;
unitario_p2_p1_y = vec_p2_p1_y ./ magnitud_p2_p1;
unitario_p2_p1_z = vec_p2_p1_z ./ magnitud_p2_p1;

% Calcular la fuerza en la direccion del vector unitario
F_resorte_x = F_resorte .* unitario_p2_p1_x;
F_resorte_y = F_resorte .* unitario_p2_p1_y;
F_resorte_z = F_resorte .* unitario_p2_p1_z;  % Componente en Z
F_negra_z = F_negra .* unitario_p2_p1_z;  % Componente en Z
F_amarilla_z = F_amarilla .* unitario_p2_p1_z;  % Componente en Z
F_verde_z = F_verde .* unitario_p2_p1_z;  % Componente en Z
F_azul_z = F_azul .* unitario_p2_p1_z;  % Componente en Z


%GRaficar la fuerza ejercida por cada banda en funcion del angulo
figure;
hold on;
plot(rad2deg(theta),(F_resorte_z) ,'Color', 'r', 'LineWidth', 2, 'DisplayName', 'Banda roja');
plot(rad2deg(theta), F_negra_z,'Color', 'k', 'LineWidth', 2, 'DisplayName', 'Banda negra');
plot(rad2deg(theta), F_azul_z, 'Color', 'b','LineWidth', 2, 'DisplayName', 'Banda azul');
plot(rad2deg(theta), F_amarilla_z,'Color', 'y', 'LineWidth', 2, 'DisplayName', 'Banda amarilla');
plot(rad2deg(theta), F_verde_z, 'Color', 'g','LineWidth', 2, 'DisplayName', 'Banda verde');
xlabel('Alfa (grados)');
ylabel('Fuerza z (N)');
title('Fuerza Z en funcion de Alfa');
grid on;
hold off;

% Graficar la componente de fuerza en Z en funcin de theta
%figure;
%plot(rad2deg(theta), F_resorte_z, 'LineWidth', 2);
%xlabel('Theta (grados)');
%ylabel('Fuerza en Z (N)');
%title('Componente de la fuerza en Z en funcin de Theta');
%grid on;


% Graficar la componente de fuerza en Z en funcin de theta
%figure;
%plot(delta_distancia, F_resorte, 'LineWidth', 2);
%xlabel('delta d');
%ylabel('Fuerza (N)');
%title('Fuerza en funcin de delta');
%grid on;

% Graficar la componente de fuerza en Z en funcin de theta
%figure;
%plot(rad2deg(theta),(p1z - p2(3)) , 'LineWidth', 2);
%xlabel('Theta');
%ylabel('Z');
%title('Distancia z en funcin de theta');
%grid on;

% Graficar la componente de fuerza en Z en funcin de theta
%figure;
%plot(rad2deg(theta),(F_resorte) , 'LineWidth', 2);
%xlabel('Theta');
%ylabel('Fuerza');
%title('Fuerza en funcin de theta');
%grid on;

%Calcular torque
torque = p1x.*F_resorte_z/1000;
torque_negro=p1x.*F_negra_z/1000;
torque_azul=p1x.*F_azul_z/1000;
torque_amarillo=p1x.*F_amarilla_z/1000;
torque_verde=p1x.*F_verde_z/1000;

%Graficar torques
figure;
hold on;
plot(rad2deg(theta),(torque) ,'Color', 'r', 'LineWidth', 2, 'DisplayName', 'Banda roja');
%plot(rad2deg(theta), torque_negro,'Color', 'k', 'LineWidth', 2, 'DisplayName', 'Banda negra');
plot(rad2deg(theta), torque_azul, 'Color', 'b','LineWidth', 2, 'DisplayName', 'Banda azul');
plot(rad2deg(theta), torque_amarillo,'Color', 'y', 'LineWidth', 2, 'DisplayName', 'Banda amarilla');
plot(rad2deg(theta), torque_verde, 'Color', 'g','LineWidth', 2, 'DisplayName', 'Banda verde');
xlabel('Alfa (�)');
ylabel('Torque (Nm)');
title('Torque en funci�n de Alfa');
grid on;
hold off;




