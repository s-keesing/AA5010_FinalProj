function N_omega_RnN = RnNomega(time)

    % define theta dot
    theta_dot = 0.000884797;

    % angular velocity in LMO frame
    omega_LMO = [0; 0; theta_dot];

    % convert to inertial frame
    N_omega_RnN = lmo2dcm(time)'*omega_LMO;
end