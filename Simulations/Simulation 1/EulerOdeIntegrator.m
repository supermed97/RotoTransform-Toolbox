%% Integrator of Dynamics Euler_dot= J* OMEGA
function [Tout,Error,Xout,Xdout,uout] = EulerOdeIntegrator(xIn,OMEGA_fn)


global  RunT Ts

ti       = 0;     %% initial time
Error    = [];    %% Error
Tout     = [];    %% Output time
Xout     = [];    %% X true out
Xdout    = [];    %% Xd out
uout     = [];    %% u out

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
        [ucont]   = fSMC(xIn, xref, t,OMEGA_fn);
        
        % dynamics
        x      = ode4(@fmodel, tspan, xIn, ucont,OMEGA_fn);

        xIn    = x(end,:)';
        x    = xIn(1:3,1);
       

       

        % plots and output signals
        e       = xref - x;        
        
        Error   = [Error e];
        Tout    = [Tout ti];   % time out
        Xout    = [Xout xIn];  % Xout (True) = [q1; q2; qdot1; qdot2]
        Xdout   = [Xdout xref];  % Xdout (Reference) = [qd1; qd2]
        uout    = [uout ucont];  % ucont = [tau1; tau2]

    end
    
end



%% Model dynamics
function dx = fmodel(t, xIn, ucont,OMEGA_fn)
    x     = xIn(1:3,1);
    J = J_euler(x(1),x(2));
    omega_vector = OMEGA_fn(t);
    euler_dot= J*omega_vector;
    dx= [euler_dot(1);    
           euler_dot(2);    
           euler_dot(3)];
end





%% Reference Signal
function [xref] = fRef(t)
    t=sum(t)/length(t);
    xref =0;
end




%% Control Input and Neural Adaptation
function [ucont] =  fSMC(x, xref,t,OMEGA_fn)
    % tracking controller
    ucont= 0;
end
    