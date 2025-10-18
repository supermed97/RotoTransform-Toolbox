%%

clear all
close all
clc
GenerateHandlers2;  % creates function handler for omega vector

global Ts  RunT  

J = eye(3);
%%%%% Intermittency %%%%%% 
Ts = 0.001;   % Sampling time  (jump between Tm and TM)
Tm = Ts*1;   % T min interval
TM = Ts*1;   % T max interval 

% Run time
N = 30/Ts;
RunT = Tm + (TM-Tm).*rand(N,1);


% Intial conditions
% Large Euler angles
phi   = 0;    % 90 deg roll
theta = pi/2;    % 90 deg pitch
psi   = 0;    % 90 deg yaw

% Compute initial rotation matrix
R_0 = eulerToSO3(phi, theta, psi)

trace_R0 = trace(R_0);
disp(['trace of R0 ', num2str(trace_R0)]);

% Initial angular velocity and desired rotation
omega0 = zeros(3,1);
Rd0 = eye(3);

% Compute initial attitude error (normalized)
att_error = 0.5*trace(eye(3) - Rd0'*R_0);
disp(['psi: ', num2str(att_error)]);

% Assemble initial state vector
x0 = [R_0(:); omega0; Rd0(:)];

% Run SO(3) integrator
[Tout_SO3, Error_SO3, Xout_SO3, Xdout_SO3, uout_SO3, norms_SO3] = SO3OdeIntegrator(x0, J);


%% Plots

LIMIT        = 0.03;

Font_x       = 20;
Font_y       = 20;
Font_Legend  = 20;
Font_Title   = 20;
Tick         = 5;
Tick_BOX     = 12;
L_Wid        = 3;

omega_x = Xout_SO3(10, :);
omega_y = Xout_SO3(11, :);
omega_z = Xout_SO3(12, :);

omega_x_ref = Xdout_SO3(1, :);
omega_y_ref = Xdout_SO3(2, :);
omega_z_ref = Xdout_SO3(3, :);

figure('Name','Attitude Tracking Error','NumberTitle','off');
plot(Tout_SO3, norms_SO3, 'b-', 'LineWidth', L_Wid);
grid on;
xlabel('Time (sec)', 'FontSize', Font_x, 'Interpreter', 'latex');
ylabel('Normalized Euclidean Distance', 'FontSize', Font_y, 'Interpreter', 'latex');
title('Attitude Tracking Error vs Time', 'FontSize', Font_Title, 'Interpreter', 'latex');

figure('Name','Angular Velocities','NumberTitle','off');

subplot(3, 1, 1)
plot(Tout_SO3, omega_x, 'b-', 'LineWidth', L_Wid)
hold on
plot(Tout_SO3, omega_x_ref, 'r--', 'LineWidth', L_Wid)
xlabel('Time (sec)', 'FontSize', Font_x, 'Interpreter', 'latex')
ylabel('$\omega_x$', 'FontSize', Font_y, 'Interpreter', 'latex')
legend({'$\omega_x$','$\omega_{x,ref}$'}, 'FontSize', Font_Legend, 'Interpreter', 'latex')
title('Angular velocity $\omega_x$', 'FontSize', Font_Title, 'Interpreter', 'latex')
grid on

subplot(3, 1, 2)
plot(Tout_SO3, omega_y, 'b-', 'LineWidth', L_Wid)
hold on
plot(Tout_SO3, omega_y_ref, 'r--', 'LineWidth', L_Wid)
xlabel('Time (sec)', 'FontSize', Font_x, 'Interpreter', 'latex')
ylabel('$\omega_y$', 'FontSize', Font_y, 'Interpreter', 'latex')
legend({'$\omega_y$','$\omega_{y,ref}$'}, 'FontSize', Font_Legend, 'Interpreter', 'latex')
title('Angular velocity $\omega_y$', 'FontSize', Font_Title, 'Interpreter', 'latex')
grid on

subplot(3, 1, 3)
plot(Tout_SO3, omega_z, 'b-', 'LineWidth', L_Wid)
hold on
plot(Tout_SO3, omega_z_ref, 'r--', 'LineWidth', L_Wid)
xlabel('Time (sec)', 'FontSize', Font_x, 'Interpreter', 'latex')
ylabel('$\omega_z$', 'FontSize', Font_y, 'Interpreter', 'latex')
legend({'$\omega_z$','$\omega_{z,ref}$'}, 'FontSize', Font_Legend, 'Interpreter', 'latex')
title('Angular velocity $\omega_z$', 'FontSize', Font_Title, 'Interpreter', 'latex')
grid on

%% Extract body torque components
Mx = uout_SO3(1, :);
My = uout_SO3(2, :);
Mz = uout_SO3(3, :);


figure('Name','Body Torques','NumberTitle','off');

subplot(3, 1, 1)
plot(Tout_SO3, Mx, 'm-', 'LineWidth', L_Wid)
xlabel('Time (sec)', 'FontSize', Font_x, 'Interpreter', 'latex')
ylabel('$M_x$', 'FontSize', Font_y, 'Interpreter', 'latex')
title('Body Torque $M_x$', 'FontSize', Font_Title, 'Interpreter', 'latex')
grid on

subplot(3, 1, 2)
plot(Tout_SO3, My, 'g-', 'LineWidth', L_Wid)
xlabel('Time (sec)', 'FontSize', Font_x, 'Interpreter', 'latex')
ylabel('$M_y$', 'FontSize', Font_y, 'Interpreter', 'latex')
title('Body Torque $M_y$', 'FontSize', Font_Title, 'Interpreter', 'latex')
grid on

subplot(3, 1, 3)
plot(Tout_SO3, Mz, 'b-', 'LineWidth', L_Wid)
xlabel('Time (sec)', 'FontSize', Font_x, 'Interpreter', 'latex')
ylabel('$M_z$', 'FontSize', Font_y, 'Interpreter', 'latex')
title('Body Torque $M_z$', 'FontSize', Font_Title, 'Interpreter', 'latex')
grid on
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%        





