function RcN = Rc2dcm(time)
    
    % define r vectors in inertial frame
    [r_LMO, ~] = hill2inertial((3396.19 + 400), 20, 30, 60 + rad2deg(0.000884797)*time);
    [r_GMO, ~] = hill2inertial(20424.2, 0, 0, 250 + rad2deg(0.0000709003)*time);
    delta_r = r_GMO - r_LMO;

    % construct basis vectors
    r1 = -delta_r/norm(delta_r);
    r2 = (cross(delta_r, [0; 0; 1]))/norm(cross(delta_r, [0; 0; 1]));
    r3 = cross(r1, r2);
    
    % construct dcm
    RcN = [r1'; r2'; r3'];
end