function Gamma = quaternionGammaMatrix(omega)
    % quaternionGammaMatrix constructs the Gamma(omega) matrix for quaternion dynamics
    % Input:
    %   omega - A 3x1 vector [Omega_x; Omega_y; Omega_z]
    % Output:
    %   Gamma - A 4x4 matrix representing Gamma(omega)

    % Ensure omega is a 3x1 vector
    if length(omega) ~= 3
        error('Input omega must be a 3x1 vector.');
    end
    
    % Extract omega components
    Omega_x = omega(1);
    Omega_y = omega(2);
    Omega_z = omega(3);
    
    % Construct the Gamma matrix
    Gamma = [0,        -Omega_x, -Omega_y, -Omega_z;
             Omega_x,  0,        Omega_z, -Omega_y;
             Omega_y, -Omega_z,  0,        Omega_x;
             Omega_z,  Omega_y, -Omega_x,  0];
end
