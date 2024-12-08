function u_dot = cal_ut(t, u, r0_x, r0_y, c, v, Z_x, Z_y, Z_dot_x, Z_dot_y)
    % Evaluate Z_x, Z_y, Z_dot_x, Z_dot_y at time t
    Z_x_t = Z_x(t);
    Z_y_t = Z_y(t);
    Z_dot_x_t = Z_dot_x(t);
    Z_dot_y_t = Z_dot_y(t);

    % Calculate |z(t) - r0|^2
    a = (Z_x_t - r0_x).^2 + (Z_y_t - r0_y).^2;

    % Calculate (z(t) - r0) . z_dot
    b = (Z_x_t - r0_x) .* Z_dot_x_t + (Z_y_t - r0_y) .* Z_dot_y_t;

    % Define the numerator and denominator for the equation
    numerator = -b * u + sqrt((b.^2) * (u^2) - ((u^2) * (v.^2) - c^2) .* a);
    denominator = a;

    % Define the equation for u_dot
    u_dot = double(numerator / denominator);
return