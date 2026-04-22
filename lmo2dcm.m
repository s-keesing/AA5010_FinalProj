function HN = lmo2dcm(time)
    
    % define orbital elements
    Omega = deg2rad(20);
    i = deg2rad(30);
    theta = deg2rad(60) + 0.000884797*time;

    % construct dcm
    HN = [cos(theta)*cos(Omega)-sin(theta)*cos(i)*sin(Omega), -sin(theta)*cos(Omega) - cos(theta)*cos(i)*sin(Omega), sin(i)*sin(Omega);
    cos(theta)*sin(Omega)+sin(theta)*cos(i)*cos(Omega), -sin(theta)*sin(Omega) + cos(theta)*cos(i)*cos(Omega), -sin(i)*cos(Omega);
    sin(theta)*sin(i), cos(theta)*sin(i), cos(i)]';
end