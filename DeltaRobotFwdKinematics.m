%% Forward Kinematics Delta Robot

clc; clear; close all;

%% -------------Delta Robot Dimensions-------------%%

rho_b = 260; % Base radius [mm]
l1 = 320; % Bicep Length [mm]
l2 = 450; % Forearm Length 
rho_p = 100; % End-Effector Radius [mm]

%---------------------------------------------------%%

% Motor Angles in degrees 
theta1 = -40; %[deg]
theta2 = -20; %[deg]
theta3 = -50; %[deg]
%-------------------------------------------------%%

%% Function Call
DeltaFwdKinematics(rho_b, rho_p,l1, l2, theta1, theta2, theta3)

%% Delta Kinematics Computation

function DeltaFwdKinematics(rho_b, rho_p,l1, l2, theta1, theta2, theta3)
    rho_A = rho_b-rho_p;
    
    % Rotation matrices            
    R2_1 = [ -1/2, -sqrt(3)/2, 0;
            sqrt(3)/2, -1/2, 0;
            0, 0, 1]; % Rotación 120°
    
    
    R3_1 = [ -1/2, sqrt(3)/2, 0;
           -sqrt(3)/2, -1/2, 0;
            0, 0, 1]; % Rotación -120°
    
    
    % Coordinate calculus of points P_Ji en SC_B1 plane system
    
    P_J1 = [0; -rho_A - l1*cosd(theta1); l1*sind(theta1)]
    P_J2 = R2_1 * [0; -rho_A - l1*cosd(theta2); l1*sind(theta2)]
    P_J3 = R3_1 * [0; -rho_A - l1*cosd(theta3); l1*sind(theta3)]
    
    % Sphere equations
    syms x y z
    eq1 = (x - P_J1(1))^2 + (y - P_J1(2))^2 + (z - P_J1(3))^2 - l2^2;
    eq2 = (x - P_J2(1))^2 + (y - P_J2(2))^2 + (z - P_J2(3))^2 - l2^2;
    eq3 = (x - P_J3(1))^2 + (y - P_J3(2))^2 + (z - P_J3(3))^2 - l2^2;
    
    sol = solve([eq1, eq2, eq3], [x, y, z]);
    
    %Seleccionar la solución con la coordenada Z más baja (más negativa)
    solutions = [double(sol.x), double(sol.y), double(sol.z)];
    [~, idx] = min(solutions(:,3)); % Tomar la solución con menor z
    o_ef = solutions(idx, :);
    
    disp("End Effector Coordinates:");
    disp(o_ef);
    
    
    %% Delta 3D graph
    
    
    % Define Base and Platform Triangle Points
    B1 = [0; -rho_b; 0];
    B2 = R2_1 * B1;
    B3 = R3_1 * B1;
    
    %Definir coordenadas de la plataforma móvil (puntos Pi)
    P1 = o_ef' + [0; -rho_p; 0];
    P2 = o_ef' + R2_1 * [0; -rho_p; 0];
    P3 = o_ef' + R3_1 * [0; -rho_p; 0];
    
    %Graphic Visualization
    figure;
    hold on; grid on; axis equal;
    xlabel('X [mm]'); ylabel('Y [mm]'); zlabel('Z [mm]');
    
    %Dibujar el centro de la base a los puntos de los actuadores
    B0 = [0; 0; 0]; % Centro de la base
    scatter3(B0(1), B0(2), B0(3), 75, 'red', 'o', 'filled')
    % Plot Base and Moving Platform Triangles
    fill3([B1(1) B2(1) B3(1)], [B1(2) B2(2) B3(2)], [B1(3) B2(3) B3(3)], 'r', 'FaceAlpha', 0.3);
    fill3([P1(1) P2(1) P3(1)], [P1(2) P2(2) P3(2)], [P1(3) P2(3) P3(3)], 'm', 'FaceAlpha', 0.3);
    plot3([B0(1) B1(1)], [B0(2) B1(2)], [B0(3) B1(3)], 'k--', 'LineWidth', 2);
    plot3([B0(1) B2(1)], [B0(2) B2(2)], [B0(3) B2(3)], 'k--', 'LineWidth', 2);
    plot3([B0(1) B3(1)], [B0(2) B3(2)], [B0(3) B3(3)], 'k--', 'LineWidth', 2);
    scatter3([B1(1), B2(1), B3(1)], [B1(2), B2(2), B3(2)], [B1(3), B2(3), B3(3)], 50, 'red', 'filled');
    
    %Brazos actuadores
    plot3([B1(1) P_J1(1)], [B1(2) P_J1(2)], [B1(3) P_J1(3)], 'black', 'LineWidth', 2);
    plot3([B2(1) P_J2(1)], [B2(2) P_J2(2)], [B2(3) P_J2(3)], 'black', 'LineWidth', 2);
    plot3([B3(1) P_J3(1)], [B3(2) P_J3(2)], [B3(3) P_J3(3)], 'black', 'LineWidth', 2);
    
    %Brazos pasivos
    plot3([P_J1(1) P1(1)], [P_J1(2) P1(2)], [P_J1(3) P1(3)], 'g', 'LineWidth', 2);
    plot3([P_J2(1) P2(1)], [P_J2(2) P2(2)], [P_J2(3) P2(3)], 'g', 'LineWidth', 2);
    plot3([P_J3(1) P3(1)], [P_J3(2) P3(2)], [P_J3(3) P3(3)], 'g', 'LineWidth', 2);
    
    %Posiciones finales de brazos pasivos
    scatter3([P1(1), P2(1), P3(1)], [P1(2), P2(2), P3(2)], [P1(3), P2(3), P3(3)], 50, 'm', 'filled');
    
    %Uniones esféricas
    scatter3([P_J1(1), P_J2(1), P_J3(1)], [P_J1(2), P_J2(2), P_J3(2)], [P_J1(3), P_J2(3), P_J3(3)], 50, 'b', 'filled');
    
    %Calcular el centro del efector final como el punto medio de P1, P2 y P3
    P_center = (P1 + P2 + P3) / 3;
    
    %Posición del efector final
    scatter3(o_ef(1), o_ef(2), o_ef(3), 75, 'm', 'filled');
    
    % Crear strings para los valores
    str_angles = sprintf('\\theta_1 = %.2f°\n\\theta_2 = %.2f°\n\\theta_3 = %.2f°', theta1, theta2, theta3);
    str_coords = sprintf('x = %.2f mm\ny = %.2f mm\nz = %.2f mm', o_ef(1), o_ef(2), o_ef(3));
    
    % Añadir cajas de texto en esquinas opuestas
    annotation('textbox', [0.15, 0.75, 0.15, 0.15], 'String', str_angles, ...
               'FontSize', 8, 'FontWeight', 'bold', 'EdgeColor', 'black', ...
               'BackgroundColor', 'white', 'LineWidth', 1.5);
    
    annotation('textbox', [0.7, 0.15, 0.15, 0.15], 'String', str_coords, ...
               'FontSize', 8, 'FontWeight', 'bold', 'EdgeColor', 'black', ...
               'BackgroundColor', 'white', 'LineWidth', 1.5);
    
    
    
    title('Delta Robot Fwd Kinematics');
    view(3);
    hold off;
end