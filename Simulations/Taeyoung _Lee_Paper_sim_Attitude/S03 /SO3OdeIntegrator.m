%% Integrator of SO3 dot -> R_dot = R[Omega]_x
function [Tout,Error,Xout,Xdout,uout,distanceHistory] = SO3OdeIntegrator(xIn,J)


global RunT Ts

ti       = 0;     %% initial time
Error    = [];    %% Error
Tout     = [];    %% Output time
Xout     = [];    %% X true out
Xdout    = [];    %% Xd out
uout     = [];    %% u out
    distanceHistory = zeros(length(RunT), 1);
    for tk = 1:length(RunT)

    %     tspan = [ts ts+RunT(j)];
    %     
    %     % flows
    %     options = odeset('RelTol',1e-3,'MaxStep',.1);
    %     [t,x] = ode45(@flow,tspan,xIn,options);
        tspan = ti : Ts : ti+RunT(tk);    %%  tspan start at ts with 0.01 increment up to random number for ODE

        % time at last sample
        ti            = tspan(end);
        t      = tspan';
        % Control
        [xref]          = fRef(ti);
        %  xref    = xref(1:2,1);
             
        % Control
        [ucont]   = bodyTorque(xIn,ti,J);
        
        
        % dynamics
        x      = ode4(@fmodel, tspan, xIn, ucont,ti,J);
        
        xIn    = x(end,:)';
        x    = xIn(1:9,1);

        R = reshape(xIn(1:9), [3, 3]);
        Rd = reshape(xIn(13:21), [3, 3]);
        omega_d = get_desired_angular_velocity(ti);
        distance = NormalizedEuclideanDistance(Rd'*R);
        distanceHistory(tk) = distance;
       
       
        % % plots and output signals
        % e       = xref - x;        
        
        % Error   = [Error e];
        Tout    = [Tout ti];   % time out
        Xout    = [Xout xIn];  % 
        Xdout   = [Xdout omega_d];  %
        uout    = [uout ucont];  % 
    end
    
end
%% Unpack xIn into rotation matrices and angular velocities 

function [R, omega, Rd] = unpack(xIn)
    % Typical x1 in any 2nd order system : First 9 elements of xIn  → 3×3 rotation matrix
    R = reshape(xIn(1:9), [3, 3]);

    % x2: Last 3 elements → 3×1 angular velocity vector
    omega = xIn(10:12);

    % if xIn contains reference rotation & angular velocity
    if length(xIn) >= 21  % 9+3+9 elements for Rd and 3 for omega_d
        Rd = reshape(xIn(13:21), [3, 3]);
        
    else
        Rd = [];
    end
end

%% Model dynamics
function dx = fmodel(t, xIn, M,ti,J)

    [R, omega, Rd] = unpack(xIn);

    omega_skew =skewSymmetricMatrix(omega);
    Rdot = R * omega_skew;
    omega_dot= J \( -omega_skew * J*omega  + M);
    % Desired Attitude Integration from omega_d

    omega_d = get_desired_angular_velocity(ti);       % get reference angular velocity
    Rd_dot = Rd * skewSymmetricMatrix(omega_d);
    dx = [Rdot(:); omega_dot; Rd_dot(:)]; 
end





%% Reference Signal
function [xref] = fRef(t)
    t=sum(t)/length(t);
    xref =0;
end

%% desired omega

function [omega_d, omega_d_dot] = get_desired_angular_velocity(t)
% GET_DESIRED_ANGULAR_VELOCITY Returns desired angular velocity and its derivative
%
% Input:
%   t - current time
%
% Output:
%   omega_d - desired angular velocity [3x1 vector]
%   omega_d_dot - time derivative of desired angular velocity [3x1 vector]

    % Desired angular velocity components
    omega_d =  [0.3 * sin(0.8422 * t);
               0.21 * sin(0.3682 * t + pi);
               0.15 * sin(1.4516 * t + pi/3)];

    % Time derivative of desired angular velocity
    omega_d_dot = [0.3 * 0.8422 * cos(0.8422 * t);
                   0.21 * 0.3682 * cos(0.3682 * t + pi);
                   0.15 * 1.4516 * cos(1.4516 * t + pi/3)];
end



%% Control Input 
function [M] =  bodyTorque(x,t,J)
    
     % control gains
     kR =  3; kOmg =3;

    % Desired omega_d
    [omega_d, omega_d_dot] = get_desired_angular_velocity(t);

    % unpack states
    [R, omega, Rd] = unpack(x);

    % error on SO(3)
    R_e = Rd'*R;

    % eR
    e_R_skew = 0.5*(R_e - R_e');
    e_R =vex(e_R_skew);

    %eOmga
    e_omega=(omega - (R_e' * omega_d) );
    
    omega_skewsym = skewSymmetricMatrix(omega);

    %delta
    delta=-J*( omega_skewsym *R_e'*omega_d -R_e'*omega_d_dot  );
    
    M= -kR*e_R - kOmg*e_omega + omega_skewsym*J*omega +delta;
end
    