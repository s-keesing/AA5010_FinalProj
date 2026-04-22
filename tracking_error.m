function [sigma_BR, B_omega_BR] = tracking_error(time, sigma_BN, B_omega_BN, RN, N_omega_RN)
    
    % construct [BN]
    s1 = sigma_BN(1);
    s2 = sigma_BN(2);
    s3 = sigma_BN(3);
    s = norm(sigma_BN);
    BN = 1/(1 + s^2)^2*[4*(s1^2 - s2^2 - s3^2) + (1 - s^2)^2, 8*s1*s2 + 4*s3*(1-s^2), 8*s1*s3 - 4*s2*(1-s^2);
                        8*s2*s1 - 4*s3*(1-s^2), 4*(-s1^2 + s2^2 - s3^2) + (1 - s^2)^2, 8*s2*s3 + 4*s1*(1-s^2);
                        8*s3*s1 + 4*s2*(1-s^2), 8*s3*s2 - 4*s1*(1-s^2), 4*(-s1^2 - s2^2 + s3^2) + (1 - s^2)^2];

    % compute B_omega_BR
    B_omega_BR = B_omega_BN - BN*N_omega_RN;

    % construct relative attitude quaternion from BR
    BR = BN*RN';
    q_BR = rotm2quat(BR');

    % convert quaternion to mrp
    sigma_BR = q_BR(2:4)'/(1 + q_BR(1));


end