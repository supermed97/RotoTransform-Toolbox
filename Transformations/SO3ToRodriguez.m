function rho = SO3ToRodriguez(R)
    % Identity matrix I_3 (3x3)
    I_3 = eye(3);
    
    % Compute the inverse transformation more efficiently using \
    temp_matrix = (R - I_3) / (R + I_3);
    
    % Extract the vector using vex()
    rho = vex(temp_matrix);
end
