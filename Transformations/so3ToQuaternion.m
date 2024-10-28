function Q = so3ToQuaternion(R)
    % Check if R is a 3x3 matrix
    assert(all(size(R) == [3, 3]), 'Input R must be a 3x3 matrix.');

    % Calculate q0
    q0 = 0.5 * sqrt(1 + R(1,1) + R(2,2) + R(3,3));
    
    % Avoid division by zero if q0 is very small
    if q0 < 1e-8
        error('q0 is too small; the rotation matrix may be invalid.');
    end

    % Calculate the remaining quaternion components
    q1 = (R(3,2) - R(2,3)) / (4 * q0);
    q2 = (R(1,3) - R(3,1)) / (4 * q0);
    q3 = (R(2,1) - R(1,2)) / (4 * q0);

    % Combine into the quaternion
    Q = [q0; q1; q2; q3];
end
