% Function to compute the skew-symmetric matrix from a vector
function S = skewSymmetricMatrix(a)
    % Input: a is a 3x1 vector
    % Output: S is a 3x3 skew-symmetric matrix
    S = [  0   -a(3)  a(2);
          a(3)   0   -a(1);
         -a(2)  a(1)   0  ];
    
    % Display the resulting skew-symmetric matrix
    % disp('Skew-symmetric matrix [a]x:');
    % disp(S);
end


