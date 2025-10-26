function q = eulerToQuat(roll, pitch, yaw)
% EULER2QUAT Convert Euler angles to quaternion (Z-Y-X convention)
%
% Input:
%   roll  - Rotation around X-axis (radians)
%   pitch - Rotation around Y-axis (radians) 
%   yaw   - Rotation around Z-axis (radians)
%
% Output:
%   q - Quaternion as column vector [q0; q1; q2; q3] where:
%       q0 = scalar (real) part
%       q1, q2, q3 = vector (imaginary) parts
%
% Convention: Z-Y-X (yaw-pitch-roll) applied in order: yaw -> pitch -> roll

% Compute half angles
hr = roll / 2;
hp = pitch / 2;
hy = yaw / 2;

% Compute trigonometric functions
cr = cos(hr);
sr = sin(hr);
cp = cos(hp);
sp = sin(hp);
cy = cos(hy);
sy = sin(hy);

% Compute quaternion components
q0 = cr * cp * cy + sr * sp * sy;
q1 = sr * cp * cy - cr * sp * sy;
q2 = cr * sp * cy + sr * cp * sy;
q3 = cr * cp * sy - sr * sp * cy;

% Return as column vector [q0; q1; q2; q3]
q = [q0; q1; q2; q3];

end