function N_omega_RcN = RcNomega(time)

    % define RcN and RcNdot via numerical differencing
    RcN = Rc2dcm(time);
    RcN_dot = (Rc2dcm(time + 1e-3) - Rc2dcm(time - 1e-3))./2e-3;

    % find omega_skew
    omega_skew = -RcN_dot*RcN';

    % extract omega values from skew matrix
    Rc_omega_RcN = [-omega_skew(2, 3); omega_skew(1, 3); -omega_skew(1, 2)];

    % put in inertial frame
    N_omega_RcN = RcN'*Rc_omega_RcN;
end