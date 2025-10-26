function qInv = quatInvUnit(q)
%quatInvUnit  Inverse of a unit quaternion (scalar-first)
%   qInv = quatInvUnit(q)
%   For unit quaternions q = [q0; q1; q2; q3], qInv == qConj == [q0; -qv].
%
%   Input q may be 4x1 or 1x4. Output is a 4x1 column.

q = q(:);
if numel(q) ~= 4
    error('Input must be a 4-element quaternion [q0; q1; q2; q3].');
end

% quick non-strict check for unitarity (warn but still return conjugate)
if abs(norm(q) - 1) > 1e-8
    warning('Input quaternion is not unit-length. Returned vector is the conjugate, not the exact inverse.');
end

% conjugate (and inverse for unit quaternions)
qInv = [ q(1); -q(2); -q(3); -q(4) ];
end
