close all
clearvars

% Define time span
ti = 0;
tf = 10;

% Define Z_x(t), Z_y(t), and Z_z(t)
syms t Z_x(t) Z_y(t) Z_z(t)
Z_x(t) = sin(t);           % X-component of Z(t)
Z_y(t) = cos(t) + 10;      % Y-component of Z(t)
Z_z(t) = 2 * t;                % Z-component of Z(t) (linear increase)

% Compute derivatives
Z_dot_x = diff(Z_x, t);
Z_dot_y = diff(Z_y, t);
Z_dot_z = diff(Z_z, t);

% Assign numeric values to parameters
u0 = 0;            % Initial condition for u(t)
r0_x = 0;          % Reference position x-coordinate
r0_y = 0;          % Reference position y-coordinate
r0_z = 0;          % Reference position z-coordinate
c = 2;             % Speed of the predator
v = 1;             % Speed of the target
r_initial = [0, 0, 0]; % Initial position of the predator

% Solve the differential equation using ode45
options = odeset('Events', @(t, u) convergence_event_3d(t, u, Z_x, Z_y, Z_z, r0_x, r0_y, r0_z), ...
                 'RelTol', 1e-10, 'AbsTol', 1e-10);
[t, u] = ode45(@(t, u) cal_ut_3d(t, u, r0_x, r0_y, r0_z, c, v, Z_x, Z_y, Z_z, Z_dot_x, Z_dot_y, Z_dot_z), ...
               [ti, tf], u0, options);

% Compute Z(t)
Z_x_vals = Z_x(t);  % Z_x evaluated at all time points
Z_y_vals = Z_y(t);
Z_z_vals = Z_z(t);

% Compute r(t) using u(t)
r_x = r_initial(1) + u .* (Z_x_vals - r0_x);
r_y = r_initial(2) + u .* (Z_y_vals - r0_y);
r_z = r_initial(3) + u .* (Z_z_vals - r0_z);

% Plot Z(t) in 3D
plot3(Z_x_vals, Z_y_vals, Z_z_vals, 'r-', 'LineWidth', 2); % Plot the trajectory of Z(t)
scatter3(Z_x_vals, Z_y_vals, Z_z_vals, 25, 'r', 'filled');
hold on;

% Plot dashed lines from Z(t) points to [0, 0, 0]
for i = 1:length(Z_x_vals)
    plot3([0, Z_x_vals(i)], [0, Z_y_vals(i)], [0, Z_z_vals(i)], 'p--'); % Dashed line to [0, 0, 0]
end

% Plot r(t) in 3D
plot3(r_x, r_y, r_z, 'b-', 'LineWidth', 2); % Plot the trajectory of r(t)
scatter3(r_x, r_y, r_z, 25, 'b', 'filled');
grid on;
xlabel('X-axis');
ylabel('Y-axis');
zlabel('Z-axis');
legend('Target (Z(t))', 'Predator (r(t))');
title('Predator-Prey Dynamics in 3D Space');
hold off;

% Supporting Functions
function u_dot = cal_ut_3d(t, u, r0_x, r0_y, r0_z, c, v, Z_x, Z_y, Z_z, Z_dot_x, Z_dot_y, Z_dot_z)
    % Evaluate Z_x, Z_y, Z_z, Z_dot_x, Z_dot_y, Z_dot_z at time t
    Z_x_t = Z_x(t);
    Z_y_t = Z_y(t);
    Z_z_t = Z_z(t);
    Z_dot_x_t = Z_dot_x(t);
    Z_dot_y_t = Z_dot_y(t);
    Z_dot_z_t = Z_dot_z(t);

    % Calculate |z(t) - r0|^2
    a = (Z_x_t - r0_x)^2 + (Z_y_t - r0_y)^2 + (Z_z_t - r0_z)^2;

    % Calculate (z(t) - r0) . z_dot
    b = (Z_x_t - r0_x) * Z_dot_x_t + (Z_y_t - r0_y) * Z_dot_y_t + (Z_z_t - r0_z) * Z_dot_z_t;

    % Define the numerator and denominator for the equation
    numerator = -b * u + sqrt((b^2) * (u^2) - ((u^2) * (v^2) - c^2) * a);
    denominator = a;

    % Define the equation for u_dot
    u_dot = double(numerator / denominator);
end


function [value, isterminal, direction] = convergence_event_3d(t, u, Z_x, Z_y, Z_z, r0_x, r0_y, r0_z)
    % Event to stop the simulation when the predator converges to the target
    Zx = double(subs(Z_x, t));
    Zy = double(subs(Z_y, t));
    Zz = double(subs(Z_z, t));
    % Compute r(t) using u(t)
    r_x = r0_x + u .* (Zx - r0_x);
    r_y = r0_y + u .* (Zy - r0_y);
    r_z = r0_z + u .* (Zz - r0_z);

    distance = sqrt((Zx - r_x)^2 + (Zy - r_y)^2 + (Zz - r_z)^2);
    value = distance - 0.1; % Stop when distance is less than a small threshold
    isterminal = 1;          % Stop the integration
    direction = 0;           % Detect all zero crossings
end
