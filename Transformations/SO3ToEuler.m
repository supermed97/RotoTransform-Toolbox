function eulerAngles = SO3ToEuler(R)
    % SO3ToEuler computes the Euler angles (phi, theta, psi) from a rotation matrix R
    % Input:
    %   R - A 3x3 rotation matrix in SO(3)
    % Output:
    %   eulerAngles - A 3x1 column vector [phi; theta; psi] where:
    %     phi   - Rotation around the x-axis (roll)
    %     theta - Rotation around the y-axis (pitch)
    %     psi   - Rotation around the z-axis (yaw)

    % Ensure R is a valid rotation matrix
    if size(R,1) ~= 3 || size(R,2) ~= 3
        error('Input must be a 3x3 matrix.');
    end

    % Compute the Euler angles
    phi = atan2(R(3,2), R(3,3));          % Roll (phi)
    theta = atan2(-R(3,1), sqrt(R(3,2)^2 + R(3,3)^2));  % Pitch (theta)
    psi = atan2(R(2,1), R(1,1));          % Yaw (psi)

    % Return the Euler angles as a 3x1 column vector
    eulerAngles = [phi; theta; psi];
end
