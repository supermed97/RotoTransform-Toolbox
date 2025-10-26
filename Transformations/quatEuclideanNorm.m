function d = quatEuclideanNorm(q)
% QUAT_NORM_DISTANCE Calculate normalized Euclidean distance for rotation matrices
% 
% This function computes the normalized Euclidean distance equivalent to
% ∥R - I∥²_F / 4 = 1 - q₀², where R is the rotation matrix and I is identity
%
% Input:
%   q - Quaternion as column vector [q0; q1; q2; q3] or row vector [q0, q1, q2, q3]
%
% Output:
%   d - Normalized Euclidean distance (1 - q0^2)
%
% Reference:
%   This represents the chordal distance between rotation matrix R and identity

% Ensure input is a column vector and extract q0

q0 = q(1);


% Calculate normalized Euclidean distance: ∥R - I∥²_F / 4 = 1 - q₀²
d = 1 - q0^2;

end