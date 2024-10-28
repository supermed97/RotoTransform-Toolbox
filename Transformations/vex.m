% Function to compute the vector from a skew-symmetric matrix
function a = vex(S)
    % Input: S is a 3x3 skew-symmetric matrix
    % Output: a is a 3x1 vector
    a = [S(3,2); S(1,3); S(2,1)];
    
    % Display the resulting vector
    % disp('Extracted vector from skew-symmetric matrix:');
    % disp(a);
end