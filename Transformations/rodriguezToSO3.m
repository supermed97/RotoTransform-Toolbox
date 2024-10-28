function R = rodriguezToSO3(rho)
    % Compute the Euclidean norm of rho
    rho_norm = norm(rho);
    
    % Identity matrix I_3 (3x3)
    I_3 = eye(3);
    
    % Skew-symmetric matrix [rho]_x
    rho_skew = skewSymmetricMatrix(rho);
    
    % Compute the rotation matrix R_rho using the given formula
    R = (1 / (1 + rho_norm^2)) * ( (1 - rho_norm^2) * I_3 + 2 * rho_skew + 2 * (rho * rho') );
end
