function Q3 = quaternionMultiply(Q1, Q2)
    % Check if Q1 and Q2 are 4x1 vectors
    assert(numel(Q1) == 4 && numel(Q2) == 4, 'Both Q1 and Q2 must be 4-element vectors.');

    % Extract scalar and vector parts of Q1 and Q2
    q01 = Q1(1);  % Scalar part of Q1
    q1 = Q1(2:4); % Vector part of Q1

    q02 = Q2(1);  % Scalar part of Q2
    q2 = Q2(2:4); % Vector part of Q2

    % Calculate the scalar part of Q3
    q03_scalar = q01 * q02 - dot(q1, q2);

    % Calculate the vector part of Q3
    q3_vector = q01 * q2 + q02 * q1 + skewSymmetricMatrix(q1) * q2;

    % Combine into a single quaternion
    Q3 = [q03_scalar; q3_vector];
end


