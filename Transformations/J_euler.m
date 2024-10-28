function J = J_euler(theta, phi)
    % J_euler computes the transformation matrix J for given Euler angles theta and phi.
    %
    % Inputs:
    %   theta - angle theta in radians
    %   phi - angle phi in radians
    %
    % Output:
    %   J - 3x3 transformation matrix
    
    % Precompute sine, cosine, and tangent values
    s_phi = sin(phi);
    c_phi = cos(phi);
    s_theta = sin(theta);
    c_theta = cos(theta);
    t_theta = tan(theta);
    
    % Construct the transformation matrix J
    J = [1, s_phi * t_theta, c_phi * t_theta;
         0, c_phi, -s_phi;
         0, s_phi / c_theta, c_phi / c_theta];
    
    % Display the transformation matrix
    % disp('Transformation matrix J:');
    % disp(J);
end
