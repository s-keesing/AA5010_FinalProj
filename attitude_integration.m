% define inertia matrix and controls
I = diag([10 5 7.5]);
u = zeros(3, 1); % [0.01; -0.01; 0.02];

% define initial state
sigma_BN0 = [0.3; -0.4; 0.5];
omega_BN0 = deg2rad([1.00; 1.75; -2.20]);
X0 = [sigma_BN0; omega_BN0];

% integrate
dynamics_anon = @(t, X) dynamics(t, X, I, u); % welcome, dynamics
[~, X] = RK4(dynamics_anon, [0 500], X0, 1);

% print results
sigma_end = X(1:3, end);
omega_end = X(4:6, end);
H = I*omega_end;
T = 1/2*omega_end'*I*omega_end;
fprintf('%.10f %.10f %.10f\n', H(1), H(2), H(3));
fprintf('%.10f\n', T);
fprintf('%.10f %.10f %.10f\n', sigma_end(1), sigma_end(2), sigma_end(3));

s1 = sigma_end(1);
s2 = sigma_end(2);
s3 = sigma_end(3);
s = norm(sigma_end);
BN = 1/(1 + s^2)^2*[4*(s1^2 - s2^2 - s3^2) + (1 - s^2)^2, 8*s1*s2 + 4*s3*(1-s^2), 8*s1*s3 - 4*s2*(1-s^2);
                        8*s2*s1 - 4*s3*(1-s^2), 4*(-s1^2 + s2^2 - s3^2) + (1 - s^2)^2, 8*s2*s3 + 4*s1*(1-s^2);
                        8*s3*s1 + 4*s2*(1-s^2), 8*s3*s2 - 4*s1*(1-s^2), 4*(-s1^2 - s2^2 + s3^2) + (1 - s^2)^2];
H_N = BN'*H;
fprintf('%.10f %.10f %.10f\n', H_N(1), H_N(2), H_N(3));

% rerun with new control law
u = [0.01; -0.01; 0.02];
[t, X] = RK4(dynamics_anon, [0 500], X0, 1);
sigma_100 = X(1:3, 101); % 100 seconds
fprintf('%.10f %.10f %.10f\n', sigma_100(1), sigma_100(2), sigma_100(3));

% define dynamics
function [Xdot] = dynamics(t, X, I, u)
    
    % unpack state
    sigma = X(1:3);
    omega = X(4:6);

    % define skew matrices
    sigma_skew = [0 -sigma(3) sigma(2); sigma(3) 0 -sigma(1); -sigma(2) sigma(1) 0];
    omega_skew = [0 -omega(3) omega(2); omega(3) 0 -omega(1); -omega(2) omega(1) 0];

    % compute derivatives
    sigma_dot = 1/4*((1 - norm(sigma)^2)*eye(3) + 2*sigma_skew + 2*(sigma*sigma'))*omega;
    omega_dot = I\(-omega_skew*I*omega + u);

    % pack for output
    Xdot = [sigma_dot; omega_dot];

end