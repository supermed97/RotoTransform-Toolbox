function Xi = Quat_XiMatrix(Q)
% XiMatrix computes the Xi(Q) matrix exactly as shown in the definition:
%
%   Xi(Q) = [ -q1  -q2  -q3
%              q0  -q3   q2
%              q3   q0  -q1
%             -q2   q1   q0 ]
%
% Input:
%   Q = [q0; q1; q2; q3]  quaternion components
%
% Output:
%   Xi = 4x3 matrix

% Extract components
q0 = Q(1);
q1 = Q(2);
q2 = Q(3);
q3 = Q(4);

% Construct Xi(Q)
Xi = [ -q1  -q2  -q3;
        q0  -q3   q2;
        q3   q0  -q1;
       -q2   q1   q0 ];
end
