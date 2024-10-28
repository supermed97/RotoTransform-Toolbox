function distance = NormalizedEuclideanDistance(R)
    % Check if R is a 3x3 matrix
    if ~isequal(size(R), [3, 3])
        error('Input matrix R must be a 3x3 matrix.');
    end
    
    % Identity matrix I_3
    I3 = eye(3);
    
    % Calculate the trace of (I3 - R)
    trace_val = trace(I3 - R);
    
    % Calculate the normalized Euclidean distance
    distance = (1 / 4) * trace_val;
    
    % Ensure the result is in [0, 1]
    if distance < 0 || distance > 1
        warning('Result is out of the expected range [0, 1]. Please check if R is a valid rotation matrix.');
    end
end
