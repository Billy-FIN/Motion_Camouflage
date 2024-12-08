close all
clearvars

% Define time span
ti = 0;
tf = 10;

% Define Z_x(t) and Z_y(t)
a = rand();
b = rand();
syms t Z_x(t) Z_y(t)
Z_x(t) = sin(t);     % X-component of Z(t)
Z_dot_x = diff(Z_x, t);
Z_y(t) = cos(t)+ 10;     % Y-component of Z(t)
Z_dot_y = diff(Z_y, t);

% Assign numeric values to the parameters
u0 = 0;            % Initial condition for u(t)
r0_x = 0;          % Reference position x-coordinate
r0_y = 0;          % Reference position y-coordinate
c = 2;             % Speed of the predator
% v = sqrt(Z_dot_x(0).^2 + Z_dot_y(0).^2); % Speed of the target
v = 1;
r_initial = [0, 0]; % Initial position of the predator


%%

% Solve the differential equation using ode45
options = odeset('Events', @(t, u) convergence_event(t, u, Z_x, Z_y, r0_x, r0_y),'RelTol',1e-10,'AbsTol',1e-10);
[t, u] = ode45(@(t, u) cal_ut(t, u, r0_x, r0_y, c, v, Z_x, Z_y, Z_dot_x, Z_dot_y), [ti, tf], u0, options);

% Compute Z(t)
Z_x_vals = Z_x(t);  % Z_x evaluated at all time points
Z_y_vals = Z_y(t);

% Compute r(t) using u(t)
r_x = r_initial(1) + u .* (Z_x_vals - r0_x);
r_y = r_initial(2) + u .* (Z_y_vals - r0_y);

% Plot Z(t)
plot(Z_x_vals, Z_y_vals, 'r-', 'LineWidth', 2); % Plot the trajectory of Z(t)
scatter(Z_x_vals, Z_y_vals, 25, 'r', 'filled');
hold on;

% Plot dashed lines from Z(t) points to [0, 0]
for i = 1:length(Z_x_vals)
    plot([0, Z_x_vals(i)], [0, Z_y_vals(i)], 'p--'); % Dashed line to [0, 0]
end

% Plot r(t)
plot(r_x, r_y, 'b-', 'LineWidth', 2); % Plot the trajectory of r(t)
scatter(r_x, r_y, 25, 'b', 'filled');
grid on;