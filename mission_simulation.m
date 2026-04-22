% define orbital constants
mu = 42828.3; % km^3/s^2

% define LMO orbit constants
r_LMO = 400 + 3396.19; % km
Omega_LMO = 20; % deg
i_LMO = 30; % deg
theta_LMO_0 = 60; % deg

% define GMO orbit constants
r_GMO = 20424.2; % km
Omega_GMO = 0; % deg
i_GMO = 0; % deg
theta_GMO_0 = 250; % deg

% define inertia matrix
I = diag([10 5 7.5]); 

% define initial states
sigma_BN0 = [0.3; -0.4; 0.5]; 
B_omega_BN0 = deg2rad([1.00; 1.75; -2.20]); 

% define control gains
P = 20/120; 
K = (P^2)/5;

% define time vector and create storage
t_sim = 0:1:6500;
Nt = length(t_sim);
X = zeros(6, Nt);
X(:, 1) = [sigma_BN0; B_omega_BN0];
sigma_BR_hist = zeros(3, Nt);
omega_BR_hist = zeros(3, Nt);
mode_hist = zeros(1, Nt);
u_hist = zeros(3, Nt);

% simulation and control loop
for k = 1:Nt-1

    % update time
    t = t_sim(k);

    % update spacecraft position and get LMO and GMO states
    theta_LMO = rad2deg(sqrt(mu/r_LMO^3)*t) + theta_LMO_0;
    [N_r_LMO, N_v_LMO] = hill2inertial(r_LMO, Omega_LMO, i_LMO, theta_LMO);

    theta_GMO = rad2deg(sqrt(mu/r_GMO^3)*t) + theta_GMO_0;
    [N_r_GMO, N_v_GMO] = hill2inertial(r_GMO, Omega_GMO, i_GMO, theta_GMO);

    % compute angle between LMO and GMO spacecraft
    comms_angle = acosd(dot(N_r_LMO, N_r_GMO)/(r_LMO*r_GMO));

    % spacecraft state logic
    if N_r_LMO(2) > 0
        scenario = 1; % sun-pointing mode
    elseif comms_angle < 35
        scenario = 2; % communication mode
    else
        scenario = 3; % nadir-pointing mode
    end

    % set rotation matrices based on scenario
    switch scenario
        case 1
            RN = inertial2sunpointing; 
            N_omega_RN = zeros(3, 1);
        case 2
            RN = Rc2dcm(t);
            N_omega_RN = RcNomega(t);
        case 3
            RN = Rn2dcm(t);
            N_omega_RN = RnNomega(t);
    end

    % compute control  
    [sigma_BR, B_omega_BR] = tracking_error(t, X(1:3, k), X(4:6, k), RN, N_omega_RN); 
    if norm(sigma_BR) >= 1
        sigma_BR = -sigma_BR/norm(sigma_BR)^2;
    end
    u = -K*sigma_BR - P*B_omega_BR; 

    % define anonymous function
    f = @(t, x) dynamics_with_control(x, I, u);

    % integrate over one timestep and save
    [~, X_int] = RK4(f, [t t+1], X(:, k), 1);
    X(:, k+1) = X_int(:, end);

    % store tracking error
    sigma_BR_hist(:, k) = sigma_BR;
    omega_BR_hist(:, k) = B_omega_BR;
    mode_hist(:, k) = scenario;
    u_hist(:, k) = u;

end

% print results at given times 
fprintf('%.10f %.10f %.10f\n', X(1, 301), X(2, 301), X(3, 301)); 
fprintf('%.10f %.10f %.10f\n', X(1, 2101), X(2, 2101), X(3, 2101)); 
fprintf('%.10f %.10f %.10f\n', X(1, 3401), X(2, 3401), X(3, 3401)); 
fprintf('%.10f %.10f %.10f\n', X(1, 4401), X(2, 4401), X(3, 4401)); 
fprintf('%.10f %.10f %.10f\n', X(1, 5601), X(2, 5601), X(3, 5601)); 

% plot 
plot_attitude_results(t_sim, X, sigma_BR_hist, omega_BR_hist)

figure
mode_hist(:, end) = nan;
plot(t_sim, mode_hist, 'LineWidth', 1.8)
xlabel('Time (s)', 'FontSize', 18, 'Interpreter', 'latex');
ylabel('Mode', 'FontSize', 18, 'Interpreter', 'latex');
title('Satellite Attitude Mode Over Time', 'FontSize', 20, 'Interpreter', 'latex');
set(groot,'defaultAxesTickLabelInterpreter', 'latex');
ylim([0.5, 3.5])
set(gca, 'FontSize', 14);

figure;
set(gcf, 'Color', 'w');
hold on;
grid on;
box on;
plot(t_sim, u_hist(1, :), 'LineWidth', 1.8);
plot(t_sim, u_hist(2, :), 'LineWidth', 1.8);
plot(t_sim, u_hist(3, :), 'LineWidth', 1.8);
xlabel('Time (s)', 'FontSize', 18, 'Interpreter', 'latex');
ylabel('Control Effort (Nm)', 'FontSize', 18, 'Interpreter', 'latex');
title('Control Effort by Axis', 'FontSize', 20, 'Interpreter', 'latex');
legend({'$u_1$', '$u_2$', '$u_3$'}, 'Location', 'best', 'Interpreter', 'latex');
set(gca, 'FontSize', 14);

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