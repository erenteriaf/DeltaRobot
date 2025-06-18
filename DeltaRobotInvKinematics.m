%% Delta Robot Inverse Kinematics

clc; clear; close all;

%% ------------- Delta Robot Dimensions -------------%%
rho_b = 260; % Base radius [mm]
l1 = 320; % Bicep Length [mm]
l2 = 450; % Forearm Length [mm]
rho_p = 100; % End-Effector Radius [mm]

% Desired End Effector Position (Input)
o_ef = [86.6106  ; 12.4951 ; -349.1644]; % [x_o, y_o, z_o] in mm
%% Function Call
[theta1, theta2, theta3, P_J1, P_J2, P_J3] = DeltaInvKinematics(rho_b, rho_p, l1, l2, o_ef);

disp("Joint Angles:");
disp([theta1, theta2, theta3]);

%% Delta Inverse Kinematics Computation
function [theta1, theta2, theta3, P_J1, P_J2, P_J3] = DeltaInvKinematics(rho_b, rho_p, l1, l2, o_ef)
    rho_A = rho_b - rho_p;

    % Rotation matrices            
    R2_1 = [ -1/2, -sqrt(3)/2, 0;
            sqrt(3)/2, -1/2, 0;
            0, 0, 1]; % Rotation 120°
    
    R3_1 = [ -1/2, sqrt(3)/2, 0;
           -sqrt(3)/2, -1/2, 0;
            0, 0, 1]; % Rotation -120°

    % Define base points
    B1 = [18.8/1000; -rho_b; 0]; % Base point for arm 1; 18.8mm traslated to the right
    B2 = R2_1 * B1;     % Base point for arm 2 (rotated 120°)
    B3 = R3_1 * B1;     % Base point for arm 3 (rotated 240°)
    
    % Define platform points relative to the end-effector position
    P1 = o_ef + [0; -rho_p; 0]; % Platform point for arm 1
    P2 = o_ef + R2_1 * [0; -rho_p; 0]; % Platform point for arm 2 (rotated 120°)
    P3 = o_ef + R3_1 * [0; -rho_p; 0]; % Platform point for arm 3 (rotated 240°)

    % Compute theta and joint positions for arm 1
    [theta1_rad, P_J1] = computeTheta(B1, P1, l1, l2, eye(3)); % No rotation for arm 1
    theta1 = rad2deg(theta1_rad); % Convert to degrees

    % Compute theta and joint positions for arm 2
    [theta2_rad, P_J2] = computeTheta(B2, P2, l1, l2, R2_1); % 120° rotation for arm 2
    theta2 = rad2deg(theta2_rad); % Convert to degrees


    % Compute theta and joint positions for arm 3
    [theta3_rad, P_J3] = computeTheta(B3, P3, l1, l2, R3_1); % 240° rotation for arm 3
    theta3 = rad2deg(theta3_rad); % Convert to degrees

    % Plot the robot
    PlotDeltaRobot(B1, B2, B3, P1, P2, P3, P_J1, P_J2, P_J3, o_ef, theta1, theta2, theta3);
end

function [theta, P_j] = computeTheta(B, P, l1, l2, R)
    % Transform P into the rotated coordinate system
    P_rot = R' * P; % Rotate P back to the y_B z_B plane
    B_rot = R' * B; % Rotate B back to the y_B z_B plane

    % Project P_rot onto the y_B z_B plane
    P_prime = [0; P_rot(2); P_rot(3)]; % Set x-coordinate to 0

    % Compute the radius of the second circle (C2)
    x_o = P_rot(1); % x-coordinate of P_rot
    phi = sqrt(l2^2 - x_o^2); % Radius of C2

    % Define the center of the first circle (C1)
    C1 = B_rot; % Center of the first circle (base joint)

    % Define the center of the second circle (C2)
    C2 = P_prime; % Center of the second circle (projected platform joint)

    % Solve for the intersection of the two circles
    [y, z] = circleIntersection(C1, C2, l1, phi);

    % Combine intersection points into a matrix
    solutions = [zeros(2, 1), y, z]; % Add x = 0 for all solutions

    % Select the solution with the smallest Y-coordinate
    [~, idx] = min(solutions(:, 2)); % Find the index of the minimum Y
    P_j_rot = solutions(idx, :)'; % Joint position in rotated coordinate system

    % Transform P_j back to the global coordinate system
    P_j = R * P_j_rot % Rotate P_j to the global coordinate system

    % Compute the angle theta using atan2 (radians)
    theta = atan2(P_j_rot(3), B_rot(2) - P_j_rot(2)); % Angle in radians
end

function [y, z] = circleIntersection(C1, C2, r1, r2)
    % Extract coordinates
    y1 = C1(2); z1 = C1(3);
    y2 = C2(2); z2 = C2(3);
    
    % Distance between centers
    d = sqrt((y2 - y1)^2 + (z2 - z1)^2);
    
    % Check if circles intersect
    if d > r1 + r2 || d < abs(r1 - r2)
        error("Circles do not intersect.");
    end
    
    % Compute intersection points
    a = (r1^2 - r2^2 + d^2) / (2 * d);
    h_squared = r1^2 - a^2;

    if h_squared < 0
        error("Invalid intersection: No real solution exists.");
    end
    h = sqrt(h_squared);

    % Intersection points
    y = (a * (y2 - y1) + h * (z2 - z1)) / d + y1;
    z = (a * (z2 - z1) - h * (y2 - y1)) / d + z1;

    y = [y; (a * (y2 - y1) - h * (z2 - z1)) / d + y1];
    z = [z; (a * (z2 - z1) + h * (y2 - y1)) / d + z1];
end

function PlotDeltaRobot(B1, B2, B3, P1, P2, P3, P_J1, P_J2, P_J3, o_ef, theta1, theta2, theta3)
    % Graphic Visualization
    figure;
    hold on; grid on; axis equal;
    xlabel('X [mm]'); ylabel('Y [mm]'); zlabel('Z [mm]');
    
    % Draw the center of the base to the actuator points
    B0 = [0; 0; 0]; % Center of the base
    scatter3(B0(1), B0(2), B0(3), 75, 'red', 'o', 'filled');

    % Plot Base and Moving Platform Triangles
    fill3([B1(1) B2(1) B3(1)], [B1(2) B2(2) B3(2)], [B1(3) B2(3) B3(3)], 'r', 'FaceAlpha', 0.3);
    fill3([P1(1) P2(1) P3(1)], [P1(2) P2(2) P3(2)], [P1(3) P2(3) P3(3)], 'm', 'FaceAlpha', 0.3);

    % Draw lines from base center to actuator points
    plot3([B0(1) B1(1)], [B0(2) B1(2)], [B0(3) B1(3)], 'k--', 'LineWidth', 2);
    plot3([B0(1) B2(1)], [B0(2) B2(2)], [B0(3) B2(3)], 'k--', 'LineWidth', 2);
    plot3([B0(1) B3(1)], [B0(2) B3(2)], [B0(3) B3(3)], 'k--', 'LineWidth', 2);

    % Scatter plot of actuator points
    scatter3([B1(1), B2(1), B3(1)], [B1(2), B2(2), B3(2)], [B1(3), B2(3), B3(3)], 50, 'red', 'filled');

    % Draw actuator arms
    plot3([B1(1) P_J1(1)], [B1(2) P_J1(2)], [B1(3) P_J1(3)], 'black', 'LineWidth', 2);
    plot3([B2(1) P_J2(1)], [B2(2) P_J2(2)], [B2(3) P_J2(3)], 'black', 'LineWidth', 2);
    plot3([B3(1) P_J3(1)], [B3(2) P_J3(2)], [B3(3) P_J3(3)], 'black', 'LineWidth', 2);

    % Draw passive arms
    plot3([P_J1(1) P1(1)], [P_J1(2) P1(2)], [P_J1(3) P1(3)], 'g', 'LineWidth', 2);
    plot3([P_J2(1) P2(1)], [P_J2(2) P2(2)], [P_J2(3) P2(3)], 'g', 'LineWidth', 2);
    plot3([P_J3(1) P3(1)], [P_J3(2) P3(2)], [P_J3(3) P3(3)], 'g', 'LineWidth', 2);

    % Scatter plot of platform points
    scatter3([P1(1), P2(1), P3(1)], [P1(2), P2(2), P3(2)], [P1(3), P2(3), P3(3)], 50, 'm', 'filled');

    % Scatter plot of spherical joints
    scatter3([P_J1(1), P_J2(1), P_J3(1)], [P_J1(2), P_J2(2), P_J3(2)], [P_J1(3), P_J2(3), P_J3(3)], 50, 'b', 'filled');

    % Calculate the center of the end-effector as the midpoint of P1, P2, and P3
    P_center = (P1 + P2 + P3) / 3;

    % Scatter plot of end-effector position
    scatter3(o_ef(1), o_ef(2), o_ef(3), 75, 'm', 'filled');

    % Create strings for the values
    str_angles = sprintf('\\theta_1 = %.2f°\n\\theta_2 = %.2f°\n\\theta_3 = %.2f°', theta1, theta2, theta3);
    str_coords = sprintf('x = %.2f mm\ny = %.2f mm\nz = %.2f mm', o_ef(1), o_ef(2), o_ef(3));

    % Add text boxes in opposite corners
    annotation('textbox', [0.15, 0.75, 0.15, 0.15], 'String', str_angles, ...
               'FontSize', 8, 'FontWeight', 'bold', 'EdgeColor', 'black', ...
               'BackgroundColor', 'white', 'LineWidth', 1.5);

    annotation('textbox', [0.7, 0.15, 0.15, 0.15], 'String', str_coords, ...
               'FontSize', 8, 'FontWeight', 'bold', 'EdgeColor', 'black', ...
               'BackgroundColor', 'white', 'LineWidth', 1.5);

    % Title and view
    title('Delta Robot Inverse Kinematics');
    view(3);
    hold off;
end