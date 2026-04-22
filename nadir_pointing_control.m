% define inertia matrix
I = diag([10 5 7.5]); 

% define initial states
sigma_BN0 = [0.3; -0.4; 0.5]; 
B_omega_BN0 = deg2rad([1.00; 1.75; -2.20]); 

% find attitude error (nadir-pointing frame) 
RnN = Rn2dcm(0); 
N_omega_RnN = RnNomega(0); 
[sigma_BR, B_omega_BR] = tracking_error(0, sigma_BN0, B_omega_BN0, RnN, N_omega_RnN); 

% define initial state 
X0 = [sigma_BN0; B_omega_BN0]; 

% define control gains
P = 20/120; 
K = (P^2)/5; 

% create storage and define time array
t = 0:1:500;
X = zeros(6, length(t));
X(:, 1) = X0;
sigma_BR_hist = zeros(3, length(t));
omega_BR_hist = zeros(3, length(t));

% integration loop
for k = 1:length(t)-1

    % compute control 
    RnN = Rn2dcm(t(k));
    N_omega_RnN = RnNomega(t(k)); 
    [sigma_BR, B_omega_BR] = tracking_error(t(k), X(1:3, k), X(4:6, k), RnN, N_omega_RnN); 
    if norm(sigma_BR) >= 1
        sigma_BR = -sigma_BR/norm(sigma_BR)^2;
    end
    u = -K*sigma_BR - P*B_omega_BR; 

    % define anonymous function
    f = @(t, x) dynamics_with_control(x, I, u);

    % integrate over one timestep and save
    [~, X_int] = RK4(f, [t(k) t(k+1)], X(:, k), 1);
    X(:, k+1) = X_int(:, end);

    % store tracking error
    sigma_BR_hist(:, k) = sigma_BR;
    omega_BR_hist(:, k) = B_omega_BR;

end

% print results at given times 
fprintf('%.10f %.10f %.10f\n', X(1, 16), X(2, 16), X(3, 16)); 
fprintf('%.10f %.10f %.10f\n', X(1, 101), X(2, 101), X(3, 101)); 
fprintf('%.10f %.10f %.10f\n', X(1, 201), X(2, 201), X(3, 201)); 
fprintf('%.10f %.10f %.10f\n', X(1, 401), X(2, 401), X(3, 401)); 

% plot
plot_attitude_results(t, X, sigma_BR_hist, omega_BR_hist)

% define dynamics 
function [Xdot] = dynamics_with_control(X, I, u) 

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