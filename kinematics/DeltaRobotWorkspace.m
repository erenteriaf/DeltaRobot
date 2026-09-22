%% Delta Robot Workspace

clc; clear; close all;

%% ------------- Delta Robot Dimensions (as built) -------------%%
rho_b = 215.8; % Base radius, center to biceps axis [mm]
l1    = 320;   % Biceps length [mm]
l2    = 476;   % Forearm length, rod-end center to center [mm]
rho_p = 100;   % End-effector radius [mm]

% Mechanical Limits (Input)
theta_max = [16, 12, 9]; % Crank angle at each limit switch, upper stop [deg]
theta_min = -Inf;        % Lower stop, not measured on the machine [deg]
beta_max  = 14;          % Rod-end swivel the M6 rod ends allow, catalogue value [deg]
det_min   = 0.15;        % Parallel singularity margin, 0 = forearms coplanar
ser_min   = 0.15;        % Serial singularity margin, 0 = arm stretched or folded

% Sweep Settings (Input)
res  = 15;                % Grid resolution [mm]
o_ef = [0; 0; -500];      % Sample pose drawn inside the cloud, in mm
                          % The board center, [-70; 100; -500], needs 14.8 deg
                          % of rod-end swivel and falls outside at beta_max = 14

%% Function Call
DeltaWorkspace(rho_b, rho_p, l1, l2, theta_max, theta_min, beta_max, det_min, ser_min, res, o_ef)

%% Delta Workspace Computation
% Keeps the points the robot can actually be commanded to, which takes five
% conditions, not just the first one:
%   1. the loop closes, so the forearm sphere reaches the arm plane and the
%      circle it leaves there cuts the biceps circle;
%   2. no crank is asked to go past its limit switch, which is the hard upper
%      stop the arms are homed against;
%   3. no forearm is swung further out of its arm plane than the rod ends at
%      its two ends can take;
%   4. no arm is stretched out or folded back in its own plane (type 1, serial
%      singularity), where the arm gains no velocity along the forearm;
%   5. the three forearms are not close to coplanar (type 2, parallel
%      singularity), where the platform loses its stiffness and the rods carry
%      the load instead of the cranks.
% Conditions 2 to 5 are the difference between the geometric envelope and the
% workspace the machine can work in, and they are not a detail: closure alone
% gives 425 L, the crank stops and the two margins bring it to 218 L, and the
% rod ends bring it to 24 L. Setting theta_max and beta_max to Inf and both
% margins to 0 gives the envelope back.
function DeltaWorkspace(rho_b, rho_p, l1, l2, theta_max, theta_min, beta_max, det_min, ser_min, res, o_ef)

    % Rotation matrices
    R2_1 = [ -1/2, -sqrt(3)/2, 0;
            sqrt(3)/2, -1/2, 0;
            0, 0, 1]; % Rotation 120°

    R3_1 = [ -1/2, sqrt(3)/2, 0;
           -sqrt(3)/2, -1/2, 0;
            0, 0, 1]; % Rotation -120°

    % Define base points
    B1 = [0; -rho_b; 0]; % Base point for arm 1
    B2 = R2_1 * B1;      % Base point for arm 2 (rotated 120°)
    B3 = R3_1 * B1;      % Base point for arm 3 (rotated 240°)

    % Search grid, one candidate end-effector position per row. The platform
    % center cannot leave this box with the arms stretched out.
    r_max = rho_b - rho_p + l1 + l2; % Farthest reach from the base axis
    z_min = -(l1 + l2);              % The robot works below its base

    [X, Y, Z] = ndgrid(-r_max:res:r_max, -r_max:res:r_max, z_min:res:0);
    P = [X(:), Y(:), Z(:)];
    P = P(hypot(P(:, 1), P(:, 2)) <= r_max, :); % Drop the corners of the box

    % Compute the solution of each arm over the whole grid
    [ok1, theta1, ~, U1, ser1, beta1] = computeTheta(P, eye(3), rho_b, rho_p, l1, l2); % No rotation for arm 1
    [ok2, theta2, ~, U2, ser2, beta2] = computeTheta(P, R2_1, rho_b, rho_p, l1, l2);   % 120° rotation for arm 2
    [ok3, theta3, ~, U3, ser3, beta3] = computeTheta(P, R3_1, rho_b, rho_p, l1, l2);   % 240° rotation for arm 3

    % 1. The loop has to close on the three arms
    usable = ok1 & ok2 & ok3;

    % 2. No crank past its limit switch
    usable = usable & theta1 <= theta_max(1) & theta1 >= theta_min;
    usable = usable & theta2 <= theta_max(2) & theta2 >= theta_min;
    usable = usable & theta3 <= theta_max(3) & theta3 >= theta_min;

    % 3. No forearm swung further out of its arm plane than the rod ends allow
    usable = usable & beta1 <= beta_max & beta2 <= beta_max & beta3 <= beta_max;

    % 4. No arm stretched out or folded back in its own plane
    usable = usable & ser1 > ser_min & ser2 > ser_min & ser3 > ser_min;

    % 5. The three forearms must not approach a common plane. Their unit
    %    vectors are the rows of the platform Jacobian, so the determinant
    %    going to zero is the parallel singularity.
    det_U = U1(:, 1) .* (U2(:, 2) .* U3(:, 3) - U2(:, 3) .* U3(:, 2)) ...
          - U1(:, 2) .* (U2(:, 1) .* U3(:, 3) - U2(:, 3) .* U3(:, 1)) ...
          + U1(:, 3) .* (U2(:, 1) .* U3(:, 2) - U2(:, 2) .* U3(:, 1));
    usable = usable & abs(det_U) > det_min;

    W       = P(usable, :);                                  % Usable positions
    theta_W = [theta1(usable), theta2(usable), theta3(usable)]; % Their crank angles
    geometric = ok1 & ok2 & ok3;                             % For comparison only

    if isempty(W)
        error("No usable points found.");
    end

    % Counting the cells that survive estimates the volume, 1 L = 1e6 mm^3
    vol     = size(W, 1) * res^3 / 1e6;
    vol_geo = sum(geometric) * res^3 / 1e6;
    r_W     = hypot(W(:, 1), W(:, 2));

    disp("Usable Points:");
    disp(size(W, 1));
    disp("Usable Volume [L]:");
    disp(vol);
    disp("Geometric Envelope [L], before the limits above:");
    disp(vol_geo);
    disp("Working Height Range [z mm]:");
    disp([max(W(:, 3)), min(W(:, 3))]);
    disp("Crank Angles Spanned [deg]:");
    disp([min(theta_W(:)), max(theta_W(:))]);
    disp("Largest Rod-End Swivel Used [deg]:");
    disp(max([beta1(usable); beta2(usable); beta3(usable)]));

    % Largest centered disc available at each height, to pick a work plane
    z_layers = (0:-100:min(W(:, 3)))';
    r_layers = zeros(size(z_layers));
    for i = 1:length(z_layers)
        layer = abs(W(:, 3) - z_layers(i)) <= res / 2;
        if any(layer)
            r_layers(i) = max(r_W(layer));
        end
    end

    disp("Usable Radius by Height [z mm, r mm]:");
    disp([z_layers, r_layers]);

    % Solve the sample pose so the robot can be drawn inside the cloud
    [pose1, th1, P_J1, ~, s1, b1] = computeTheta(o_ef', eye(3), rho_b, rho_p, l1, l2);
    [pose2, th2, P_J2, ~, s2, b2] = computeTheta(o_ef', R2_1, rho_b, rho_p, l1, l2);
    [pose3, th3, P_J3, ~, s3, b3] = computeTheta(o_ef', R3_1, rho_b, rho_p, l1, l2);

    pose_ok = pose1 && pose2 && pose3 ...
              && th1 <= theta_max(1) && th2 <= theta_max(2) && th3 <= theta_max(3) ...
              && min([th1, th2, th3]) >= theta_min ...
              && max([b1, b2, b3]) <= beta_max ...
              && min([s1, s2, s3]) > ser_min;

    % Define platform points relative to the end-effector position
    P1 = o_ef + [0; -rho_p; 0];        % Platform point for arm 1
    P2 = o_ef + R2_1 * [0; -rho_p; 0]; % Platform point for arm 2 (rotated 120°)
    P3 = o_ef + R3_1 * [0; -rho_p; 0]; % Platform point for arm 3 (rotated 240°)

    if ~pose_ok
        warning("Sample pose is outside the usable workspace, the robot is not drawn.");
    end

    % Plot the workspace, with the robot in the sample pose
    PlotDeltaWorkspace(W, B1, B2, B3, P1, P2, P3, P_J1', P_J2', P_J3', ...
                       o_ef, pose_ok, vol, res);
end

function [ok, theta, P_j, U, ser, beta] = computeTheta(P, R, rho_b, rho_p, l1, l2)
    % Transform P into the rotated coordinate system, one target per row.
    % Shifting by rho_p after the rotation is the same as rotating the platform
    % joint itself, and keeps a single copy of the grid in memory.
    P_rot = P * R;          % Rotate P back to the y_B z_B plane
    B_rot = [0, -rho_b, 0]; % B sits on the y_B axis once rotated back

    % Project P_rot onto the y_B z_B plane
    P_prime = [P_rot(:, 2) - rho_p, P_rot(:, 3)]; % Set x-coordinate to 0

    % Compute the radius of the second circle (C2)
    x_o = P_rot(:, 1);                 % x-coordinate of P_rot
    phi = sqrt(max(l2^2 - x_o.^2, 0)); % Radius of C2, zero where it is imaginary

    % Solve for the intersection of the two circles. The sweep rejects the
    % targets that do not intersect instead of stopping on them.
    [y, z, ok] = circleIntersection(B_rot(2), 0, P_prime(:, 1), P_prime(:, 2), l1, phi);
    ok = ok & l2^2 - x_o.^2 >= 0;

    % Select the solution with the smallest Y-coordinate
    [~, idx] = min(y, [], 2);                       % Index of the minimum Y
    pick = sub2ind(size(y), (1:size(y, 1))', idx);
    P_j_rot = [zeros(size(idx)), y(pick), z(pick)]; % Joint position, rotated frame

    % Transform P_j back to the global coordinate system
    P_j = P_j_rot * R'; % Rotate P_j to the global coordinate system

    % Compute the angle theta using atan2 (degrees)
    theta = atan2d(P_j_rot(:, 3), B_rot(2) - P_j_rot(:, 2));
    theta(~ok) = NaN;

    % Forearm vector, elbow to platform joint. Its length is l2 by construction,
    % so dividing by l2 gives the unit vector used for the parallel singularity.
    F_rot = [x_o, P_prime(:, 1) - P_j_rot(:, 2), P_prime(:, 2) - P_j_rot(:, 3)];
    U = (F_rot * R') / l2;

    % Serial singularity measure: the forearm against the direction the elbow
    % moves in. It goes to zero when biceps and forearm line up in the plane.
    ser = abs(F_rot(:, 2) .* sind(theta) + F_rot(:, 3) .* cosd(theta)) / l2;

    % Angle the forearm makes with its own arm plane, which is what the rod
    % ends at its two ends have to swallow. The out-of-plane leg is x_o.
    beta = asind(min(abs(x_o) / l2, 1));
end

function [y, z, ok] = circleIntersection(y1, z1, y2, z2, r1, r2)
    % Distance between centers
    d = sqrt((y2 - y1).^2 + (z2 - z1).^2);

    % Check if circles intersect
    ok = d > 0 & d <= r1 + r2 & d >= abs(r1 - r2);

    % Compute intersection points
    a = (r1^2 - r2.^2 + d.^2) ./ (2 * d);
    h_squared = r1^2 - a.^2;

    % Flag the invalid intersections, no real solution exists there
    ok = ok & h_squared >= 0;
    h = sqrt(max(h_squared, 0));

    % Intersection points, one solution per column
    y = [(a .* (y2 - y1) + h .* (z2 - z1)) ./ d + y1, ...
         (a .* (y2 - y1) - h .* (z2 - z1)) ./ d + y1];
    z = [(a .* (z2 - z1) - h .* (y2 - y1)) ./ d + z1, ...
         (a .* (z2 - z1) + h .* (y2 - y1)) ./ d + z1];
end

function PlotDeltaWorkspace(W, B1, B2, B3, P1, P2, P3, P_J1, P_J2, P_J3, o_ef, pose_ok, vol, res)
    % Graphic Visualization
    figure;
    hold on; grid on; axis equal;
    xlabel('X [mm]'); ylabel('Y [mm]'); zlabel('Z [mm]');

    % Point cloud of the usable positions, decimated to keep it responsive
    skip = max(1, ceil(size(W, 1) / 40000));
    scatter3(W(1:skip:end, 1), W(1:skip:end, 2), W(1:skip:end, 3), 4, ...
             'MarkerEdgeColor', [0.35 0.10 0.60], 'MarkerEdgeAlpha', 0.12);

    % Draw the center of the base to the actuator points
    B0 = [0; 0; 0]; % Center of the base
    scatter3(B0(1), B0(2), B0(3), 75, 'red', 'o', 'filled');

    % Plot Base Triangle
    fill3([B1(1) B2(1) B3(1)], [B1(2) B2(2) B3(2)], [B1(3) B2(3) B3(3)], 'r', 'FaceAlpha', 0.3);

    % Draw lines from base center to actuator points
    plot3([B0(1) B1(1)], [B0(2) B1(2)], [B0(3) B1(3)], 'k--', 'LineWidth', 2);
    plot3([B0(1) B2(1)], [B0(2) B2(2)], [B0(3) B2(3)], 'k--', 'LineWidth', 2);
    plot3([B0(1) B3(1)], [B0(2) B3(2)], [B0(3) B3(3)], 'k--', 'LineWidth', 2);

    % Scatter plot of actuator points
    scatter3([B1(1), B2(1), B3(1)], [B1(2), B2(2), B3(2)], [B1(3), B2(3), B3(3)], 50, 'red', 'filled');

    if pose_ok
        % Plot Moving Platform Triangle
        fill3([P1(1) P2(1) P3(1)], [P1(2) P2(2) P3(2)], [P1(3) P2(3) P3(3)], 'm', 'FaceAlpha', 0.3);

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

        % Scatter plot of end-effector position
        scatter3(o_ef(1), o_ef(2), o_ef(3), 75, 'm', 'filled');
    end

    % Create strings for the values
    str_volume = sprintf('V = %.0f L\npoints = %d\ngrid = %.0f mm', vol, size(W, 1), res);
    str_coords = sprintf('x = %.2f mm\ny = %.2f mm\nz = %.2f mm', o_ef(1), o_ef(2), o_ef(3));

    % Add text boxes in opposite corners
    annotation('textbox', [0.15, 0.75, 0.15, 0.15], 'String', str_volume, ...
               'FontSize', 8, 'FontWeight', 'bold', 'EdgeColor', 'black', ...
               'BackgroundColor', 'white', 'LineWidth', 1.5);

    annotation('textbox', [0.7, 0.15, 0.15, 0.15], 'String', str_coords, ...
               'FontSize', 8, 'FontWeight', 'bold', 'EdgeColor', 'black', ...
               'BackgroundColor', 'white', 'LineWidth', 1.5);

    % Title and view
    title('Delta Robot Usable Workspace');
    view(3);
    hold off;
end
