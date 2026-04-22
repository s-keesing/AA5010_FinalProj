function [t, y] = RK4(f, tspan, y0, h)

    % define time limits
    t0 = tspan(1);
    tf = tspan(2);

    % create time vector
    t = t0:h:tf;
    if t(end) < tf
        t = [t, tf];
    end

    % create storage
    y = zeros(length(y0), length(t));
    y(:, 1) = y0;

    % integration (runge-kutta)
    for n = 1:length(t)-1
        tn = t(n);
        yn = y(:, n);

        k1 = f(tn, yn);
        k2 = f(tn + h/2, yn + h/2*k1);
        k3 = f(tn + h/2, yn + h/2*k2);
        k4 = f(tn + h, yn + h*k3);

        y(:, n+1) = yn + (h/6)*(k1 + 2*k2 + 2*k3 + k4);

        sigma = y(1:3, n+1);
        if norm(sigma) >= 1
            y(1:3, n+1) = -sigma / norm(sigma)^2;
        end
    end
end