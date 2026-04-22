function RnN = Rn2dcm(time)
    
    % define rotation between lmo and Rn
    RnLMO = [-1 0 0; 0 1 0; 0 0 -1];
    
    % rotate
    RnN = RnLMO*lmo2dcm(time);
end