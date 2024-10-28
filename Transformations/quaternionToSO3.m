function R = quaternionToSO3(Q)
    % Check if Q is a 4x1 vector
    assert(numel(Q) == 4, 'Input Q must be a 4-element vector.');

    % Extract scalar and vector parts
    q0 = Q(1);         % Scalar part
    q = Q(2:4);        % Vector part [q1; q2; q3]

    % Compute the norm of the vector part
    q_norm = norm(q);

    % Use the existing skewSymmetricMatrix function for qx
    qx = skewSymmetricMatrix(q);

    % Calculate the rotation matrix using the given formula
    R = (q0^2 - q_norm^2) * eye(3) + 2 * q0 * qx + 2 * (q * q');
end
