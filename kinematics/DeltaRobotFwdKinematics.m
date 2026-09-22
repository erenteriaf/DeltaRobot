%% Forward Kinematics Delta Robot

clc; clear; close all;

%% -------------Delta Robot Dimensions-------------%%

rho_b = 215.8; % Base radius, center to biceps axis [mm]
l1    = 320;   % Biceps length [mm]
l2    = 476;   % Forearm length, rod-end center to center [mm]
rho_p = 100;   % End-effector radius [mm]

%---------------------------------------------------%%

% Motor Angles in degrees 
theta1 = 16; %[deg]
theta2 = 12; %[deg]
theta3 = 9;  %[deg]   (home pose at the limit switches)
%-------------------------------------------------%%

%% Function Call
DeltaFwdKinematics(rho_b, rho_p,l1, l2, theta1, theta2, theta3)

%% Delta Kinematics Computation

function DeltaFwdKinematics(rho_b, rho_p,l1, l2, theta1, theta2, theta3)
    rho_A = rho_b-rho_p;
    
    % Rotation matrices            
    R2_1 = [ -1/2, -sqrt(3)/2, 0;
            sqrt(3)/2, -1/2, 0;
            0, 0, 1]; % Rotation 120°
    
    
    R3_1 = [ -1/2, sqrt(3)/2, 0;
           -sqrt(3)/2, -1/2, 0;
            0, 0, 1]; % Rotation -120°
    
    
    % Elbow points P_Ji', shifted inward by rho_p so the three forearm spheres meet at o_ef
    
    P_J1 = [0; -rho_A - l1*cosd(theta1); l1*sind(theta1)];
    P_J2 = R2_1 * [0; -rho_A - l1*cosd(theta2); l1*sind(theta2)];
    P_J3 = R3_1 * [0; -rho_A - l1*cosd(theta3); l1*sind(theta3)];
    
    % Sphere equations
    syms x y z
    eq1 = (x - P_J1(1))^2 + (y - P_J1(2))^2 + (z - P_J1(3))^2 - l2^2;
    eq2 = (x - P_J2(1))^2 + (y - P_J2(2))^2 + (z - P_J2(3))^2 - l2^2;
    eq3 = (x - P_J3(1))^2 + (y - P_J3(2))^2 + (z - P_J3(3))^2 - l2^2;
    
    sol = solve([eq1, eq2, eq3], [x, y, z]);
    
    % Keep the solution below the base (most negative z)
    solutions = [double(sol.x), double(sol.y), double(sol.z)];
    [~, idx] = min(solutions(:,3)); % lowest z
    o_ef = solutions(idx, :);
    
    disp("End Effector Coordinates:");
    disp(o_ef);
    
    
    %% Delta 3D graph
    
    
    % Define Base and Platform Triangle Points
    B1 = [0; -rho_b; 0];
    B2 = R2_1 * B1;
    B3 = R3_1 * B1;

    % Physical elbow positions (undo the rho_p shift used for the sphere centers)
    E1 = P_J1 + [0; -rho_p; 0];
    E2 = P_J2 + R2_1 * [0; -rho_p; 0];
    E3 = P_J3 + R3_1 * [0; -rho_p; 0];
    
    % Moving platform points P_i
    P1 = o_ef' + [0; -rho_p; 0];
    P2 = o_ef' + R2_1 * [0; -rho_p; 0];
    P3 = o_ef' + R3_1 * [0; -rho_p; 0];
    
    % Graphic Visualization
    figure;
    hold on; grid on; axis equal;
    xlabel('X [mm]'); ylabel('Y [mm]'); zlabel('Z [mm]');
    
    % Base center to actuator points
    B0 = [0; 0; 0]; % Base center
    scatter3(B0(1), B0(2), B0(3), 75, 'red', 'o', 'filled')
    % Plot Base and Moving Platform Triangles
    fill3([B1(1) B2(1) B3(1)], [B1(2) B2(2) B3(2)], [B1(3) B2(3) B3(3)], 'r', 'FaceAlpha', 0.3);
    fill3([P1(1) P2(1) P3(1)], [P1(2) P2(2) P3(2)], [P1(3) P2(3) P3(3)], 'm', 'FaceAlpha', 0.3);
    plot3([B0(1) B1(1)], [B0(2) B1(2)], [B0(3) B1(3)], 'k--', 'LineWidth', 2);
    plot3([B0(1) B2(1)], [B0(2) B2(2)], [B0(3) B2(3)], 'k--', 'LineWidth', 2);
    plot3([B0(1) B3(1)], [B0(2) B3(2)], [B0(3) B3(3)], 'k--', 'LineWidth', 2);
    scatter3([B1(1), B2(1), B3(1)], [B1(2), B2(2), B3(2)], [B1(3), B2(3), B3(3)], 50, 'red', 'filled');
    
    % Actuated arms (biceps), drawn to the physical elbows
    plot3([B1(1) E1(1)], [B1(2) E1(2)], [B1(3) E1(3)], 'black', 'LineWidth', 2);
    plot3([B2(1) E2(1)], [B2(2) E2(2)], [B2(3) E2(3)], 'black', 'LineWidth', 2);
    plot3([B3(1) E3(1)], [B3(2) E3(2)], [B3(3) E3(3)], 'black', 'LineWidth', 2);
    
    % Passive arms (forearms)
    plot3([E1(1) P1(1)], [E1(2) P1(2)], [E1(3) P1(3)], 'g', 'LineWidth', 2);
    plot3([E2(1) P2(1)], [E2(2) P2(2)], [E2(3) P2(3)], 'g', 'LineWidth', 2);
    plot3([E3(1) P3(1)], [E3(2) P3(2)], [E3(3) P3(3)], 'g', 'LineWidth', 2);
    
    % Platform joints
    scatter3([P1(1), P2(1), P3(1)], [P1(2), P2(2), P3(2)], [P1(3), P2(3), P3(3)], 50, 'm', 'filled');
    
    % Spherical joints (elbows)
    scatter3([E1(1), E2(1), E3(1)], [E1(2), E2(2), E3(2)], [E1(3), E2(3), E3(3)], 50, 'b', 'filled');
    
    % Platform center
    P_center = (P1 + P2 + P3) / 3;
    
    % End-effector position
    scatter3(o_ef(1), o_ef(2), o_ef(3), 75, 'm', 'filled');
    
    % Annotation strings
    str_angles = sprintf('\\theta_1 = %.2f°\n\\theta_2 = %.2f°\n\\theta_3 = %.2f°', theta1, theta2, theta3);
    str_coords = sprintf('x = %.2f mm\ny = %.2f mm\nz = %.2f mm', o_ef(1), o_ef(2), o_ef(3));
    
    % Text boxes in opposite corners
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