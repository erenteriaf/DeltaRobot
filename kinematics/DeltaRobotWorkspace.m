%% Delta Robot Workspace Volume
%
% Maps the reachable workspace of the delta. A Cartesian grid is swept, every
% point is tested against the closed-form inverse kinematics, and the points
% that survive are drawn as a cloud around the robot in a sample pose.
%
% A point is reachable when all three arms have a real IK solution:
%   1) the forearm sphere reaches the arm plane, |x_i| <= l2, cutting it in a
%      circle of radius  phi_i = sqrt(l2^2 - x_i^2);
%   2) that circle intersects the biceps circle of radius l1 around the
%      shoulder,  |l1 - phi_i| <= d_i <= l1 + phi_i;
%   3) the resulting crank angle theta_i lies inside theta_lim. This is left
%      unbounded by default, so the cloud is the purely geometric workspace;
%      set theta_lim to the mechanical travel to cut it down to the usable one.
%
% The number of surviving cells times the cell volume estimates the reachable
% volume. Same dimensions and conventions as DeltaRobotInvKinematics.m.

clc; clear; close all;

%% ------------- Delta Robot Dimensions (as built) -------------%%
rho_b = 215.8; % Base radius, center to biceps axis [mm]
l1    = 320;   % Biceps length [mm]
l2    = 476;   % Forearm length, rod-end center to center [mm]
rho_p = 100;   % End-effector radius [mm]

%% ------------- Sweep Settings -------------%%
res        = 15;               % Grid resolution [mm], drives cost and detail
theta_lim  = [-Inf, Inf];      % Allowed crank angles [deg], e.g. [-75, 15]
o_pose     = [-70; 100; -500]; % Sample pose drawn inside the cloud [mm]
show_slice = true;             % Also plot the x-z cross section at y = 0
max_plot   = 40000;            % Cloud points drawn (the rest are decimated)

%% ------------- Search Grid -------------%%
% Farthest the platform center can sit from the base axis, arms stretched out
r_max = rho_b - rho_p + l1 + l2;
z_min = -(l1 + l2);            % The robot works below its base, so z <= 0

xs = -r_max:res:r_max;
zs = z_min:res:0;
[X, Y, Z] = ndgrid(xs, xs, zs);
P = [X(:), Y(:), Z(:)];
P = P(hypot(P(:,1), P(:,2)) <= r_max, :); % Drop the corners of the box

%% ------------- Reachability Test -------------%%
% Rotation matrices
R2_1 = [ -1/2, -sqrt(3)/2, 0;
        sqrt(3)/2, -1/2, 0;
        0, 0, 1]; % Rotation 120°

R3_1 = [ -1/2, sqrt(3)/2, 0;
       -sqrt(3)/2, -1/2, 0;
        0, 0, 1]; % Rotation -120°

Rot = {eye(3), R2_1, R3_1};

reachable = true(size(P, 1), 1);
Theta     = zeros(size(P, 1), 3);

for i = 1:3
    % Rows of P * Rot{i} are the targets rotated into arm i's y_B z_B plane
    [ok_i, theta_i] = ArmSolution(P * Rot{i}, rho_b, rho_p, l1, l2);
    reachable = reachable & ok_i & theta_i >= theta_lim(1) & theta_i <= theta_lim(2);
    Theta(:, i) = theta_i;
end

W     = P(reachable, :);     % Reachable end-effector positions
W_th  = Theta(reachable, :); % Their crank angles
n_pts = size(W, 1);

if n_pts == 0
    error("No reachable points found. Check the dimensions or theta_lim.");
end

%% ------------- Reported Figures -------------%%
vol_L = n_pts * res^3 / 1e6;   % 1 L = 1e6 mm^3
r_W   = hypot(W(:,1), W(:,2));

fprintf("Grid resolution:      %.1f mm (%d points tested)\n", res, size(P,1));
fprintf("Reachable points:     %d\n", n_pts);
fprintf("Estimated volume:     %.1f L\n", vol_L);
fprintf("Height range:         z = %.1f to %.1f mm\n", min(W(:,3)), max(W(:,3)));
fprintf("Max horizontal reach: r = %.1f mm\n", max(r_W));
fprintf("Crank angles spanned: %.1f to %.1f deg\n", min(W_th(:)), max(W_th(:)));

% Largest centered disc available at each height, useful to pick a work plane
fprintf("\n  z [mm]   usable radius [mm]\n");
for z_q = -300:-100:min(W(:,3))
    layer = abs(W(:,3) - z_q) <= res/2;
    if any(layer)
        fprintf("  %6.0f   %6.0f\n", z_q, max(r_W(layer)));
    end
end

%% ------------- Sample Pose -------------%%
B  = [[0; -rho_b; 0], R2_1 * [0; -rho_b; 0], R3_1 * [0; -rho_b; 0]]; % Shoulders
Pp = [o_pose + [0; -rho_p; 0], ...
      o_pose + R2_1 * [0; -rho_p; 0], ...
      o_pose + R3_1 * [0; -rho_p; 0]];                               % Platform joints
E  = zeros(3, 3);                                                    % Elbows
pose_ok = true;

for i = 1:3
    [ok_i, th_i, y_J, z_J] = ArmSolution((Rot{i}' * o_pose)', rho_b, rho_p, l1, l2);
    pose_ok = pose_ok && ok_i && th_i >= theta_lim(1) && th_i <= theta_lim(2);
    E(:, i) = Rot{i} * [0; y_J; z_J];
end

if ~pose_ok
    warning("Sample pose [%g %g %g] is out of the workspace, robot not drawn.", o_pose);
end

%% ------------- Workspace Plot -------------%%
figure;
hold on; grid on; axis equal;
xlabel('X [mm]'); ylabel('Y [mm]'); zlabel('Z [mm]');

% Point cloud of reachable positions, decimated so the figure stays responsive
skip = max(1, ceil(n_pts / max_plot));
cloud = scatter3(W(1:skip:end,1), W(1:skip:end,2), W(1:skip:end,3), 4, ...
                 'MarkerEdgeColor', [0.35 0.10 0.60], 'MarkerEdgeAlpha', 0.12);

if pose_ok
    PlotDeltaPose(B, E, Pp, o_pose);
end

title(sprintf('Delta Robot Workspace (%.1f L, %.0f mm grid)', vol_L, res));
legend(cloud, 'Reachable end-effector positions', 'Location', 'northeast');
view(3);
hold off;

%% ------------- Cross Section -------------%%
if show_slice
    layer = abs(W(:,2)) <= res/2; % Points on the y = 0 plane
    figure;
    hold on; grid on; axis equal;
    scatter(W(layer,1), W(layer,3), 6, [0.35 0.10 0.60], 'filled');
    plot([B(1,:), B(1,1)], [B(3,:), B(3,1)], 'r-', 'LineWidth', 2);
    if pose_ok
        scatter(o_pose(1), o_pose(3), 75, 'm', 'filled');
    end
    xlabel('X [mm]'); ylabel('Z [mm]');
    title('Workspace cross section at y = 0');
    hold off;
end

%% ------------- Arm Solution (vectorized IK feasibility) -------------%%
function [ok, theta, y_J, z_J] = ArmSolution(Q, rho_b, rho_p, l1, l2)
    % Q holds one target per row, already rotated into the arm's y_B z_B plane.
    % Returns whether that arm can reach it, the crank angle, and the elbow
    % position in the plane (outward elbow, the smallest-y intersection).

    x_o = Q(:, 1);          % Out-of-plane offset
    y_2 = Q(:, 2) - rho_p;  % Platform joint projected on the plane
    z_2 = Q(:, 3);
    y_1 = -rho_b;           % Shoulder
    z_1 = 0;

    phi_sq = l2^2 - x_o.^2;               % Squared radius of the forearm circle
    phi    = sqrt(max(phi_sq, 0));
    d      = hypot(y_2 - y_1, z_2 - z_1); % Distance between circle centers

    ok = phi_sq >= 0 & d > 0 & d <= l1 + phi & d >= abs(l1 - phi);

    % Circle intersection
    a   = (l1^2 - phi.^2 + d.^2) ./ (2 * d);
    h   = sqrt(max(l1^2 - a.^2, 0));
    u_y = (y_2 - y_1) ./ d;  u_z = (z_2 - z_1) ./ d; % Unit vector between centers

    y_A = y_1 + a .* u_y + h .* u_z;  z_A = z_1 + a .* u_z - h .* u_y;
    y_B = y_1 + a .* u_y - h .* u_z;  z_B = z_1 + a .* u_z + h .* u_y;

    % Keep the outward configuration, the solution with the smallest Y
    first     = y_A <= y_B;
    y_J       = y_B;          z_J       = z_B;
    y_J(first) = y_A(first);  z_J(first) = z_A(first);

    theta = atan2d(z_J, y_1 - y_J);
    theta(~ok) = NaN;
end

%% ------------- Robot Drawing -------------%%
function PlotDeltaPose(B, E, Pp, o_ef)
    % Base triangle, arms and platform, same colors as the kinematics scripts
    fill3(B(1,:), B(2,:), B(3,:), 'r', 'FaceAlpha', 0.3);
    fill3(Pp(1,:), Pp(2,:), Pp(3,:), 'm', 'FaceAlpha', 0.3);
    scatter3(B(1,:), B(2,:), B(3,:), 50, 'red', 'filled');
    scatter3(0, 0, 0, 75, 'red', 'filled');

    for i = 1:3
        plot3([0 B(1,i)], [0 B(2,i)], [0 B(3,i)], 'k--', 'LineWidth', 2);
        plot3([B(1,i) E(1,i)], [B(2,i) E(2,i)], [B(3,i) E(3,i)], 'black', 'LineWidth', 2);
        plot3([E(1,i) Pp(1,i)], [E(2,i) Pp(2,i)], [E(3,i) Pp(3,i)], 'g', 'LineWidth', 2);
    end

    scatter3(E(1,:), E(2,:), E(3,:), 50, 'b', 'filled');
    scatter3(Pp(1,:), Pp(2,:), Pp(3,:), 50, 'm', 'filled');
    scatter3(o_ef(1), o_ef(2), o_ef(3), 75, 'm', 'filled');
end
