function R = eulerToSO3(phi, theta, psi)
    % eulerToSO3 computes the rotation matrix R from Euler angles
    % Input:
    %   phi   - Rotation around the x-axis (roll)
    %   theta - Rotation around the y-axis (pitch)
    %   psi   - Rotation around the z-axis (yaw)
    % Output:
    %   R - The corresponding 3x3 rotation matrix in SO(3)

    % Precompute the sine and cosine values for efficiency
    cphi = cos(phi);  % cos(phi)
    sphi = sin(phi);  % sin(phi)
    ctheta = cos(theta);  % cos(theta)
    stheta = sin(theta);  % sin(theta)
    cpsi = cos(psi);  % cos(psi)
    spsi = sin(psi);  % sin(psi)
    
    % Define the rotation matrix
    R = [ ctheta*cpsi, -cphi*spsi + sphi*stheta*cpsi, sphi*spsi + cphi*stheta*cpsi;
          ctheta*spsi, cphi*cpsi + sphi*stheta*spsi, -sphi*cpsi + cphi*stheta*spsi;
          -stheta,     sphi*ctheta,                 cphi*ctheta              ];
end
