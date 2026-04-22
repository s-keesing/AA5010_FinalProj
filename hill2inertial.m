function [pos_inertial, vel_inertial] = hill2inertial(r, Omega_deg, i_deg, theta_deg)

    % define constants
    mu = 42828.3; % km^3/s^2
    n = sqrt(mu/r^3);

    % define angles in radians
    Omega_rad = deg2rad(Omega_deg);
    i_rad = deg2rad(i_deg);
    theta_rad = deg2rad(theta_deg);
    
    % create dcm
    ON = angle2dcm(Omega_rad, i_rad, theta_rad, 'ZXZ');
    NO = ON';

    % calculate inertial position
    pos_inertial = NO*[r; 0; 0];

    % angular velocity in Hill frame
    omega_O = [0; 0; n];

    % velocity (transport theorem)
    vel_inertial = NO*(cross(omega_O, [r; 0; 0]));
end