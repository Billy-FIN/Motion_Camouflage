function [value, isterminal, direction] = convergence_event(t, u, Z_x, Z_y, r0_x, r0_y)
    % Compute Z(t) at the current time
    try
        Z_x_t = double(Z_x(t)); % Ensure Z_x is evaluated numerically
        Z_y_t = double(Z_y(t)); % Ensure Z_y is evaluated numerically
    catch
        error('Error evaluating Z_x or Z_y at time t = %.2f.', t);
    end

    % Compute r(t) at the current time
    r_x_t = r0_x + u * (Z_x_t - r0_x);
    r_y_t = r0_y + u * (Z_y_t - r0_y);

    % Compute the distance between r(t) and Z(t)
    distance = sqrt((r_x_t - Z_x_t)^2 + (r_y_t - Z_y_t)^2);

    % Dynamic threshold (optional: here, constant threshold is kept)
    threshold = 0.1;

    % Event condition
    value = distance - threshold; % Stop when distance < threshold
    % disp(['t = ', num2str(t), ', value = ', num2str(value)]);
    isterminal = 1; % Stop the integration
    direction = 1;  % Detect all zero crossings
end
