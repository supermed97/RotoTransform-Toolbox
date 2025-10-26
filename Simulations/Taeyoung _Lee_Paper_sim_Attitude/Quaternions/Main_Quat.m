%%

clear all
close all
clc

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

% Compute initial conditoions
q_0 = eulerToQuat(phi, theta, psi);
omega0 = zeros(3,1);

% trace_R0 = trace(R_0);
% disp(['trace of R0 ', num2str(trace_R0)]);

% Initial angular velocity and desired rotation

qd0 = [1;0;0;0];

% Compute initial attitude error (normalized)
% att_error = 0.5*trace(eye(3) - Rd0'*R_0);
% disp(['psi: ', num2str(att_error)]);

% Assemble initial state vector
x0 = [q_0; omega0; qd0];

% Run SO(3) integrator
[Tout_Quat, Error_Quat, Xout_Quat, Xdout_Quat, uout_Quat, norms_Quat] = Quat_Attitude_Integrator(x0, J);


%% Plots

LIMIT        = 0.03;

Font_x       = 20;
Font_y       = 20;
Font_Legend  = 20;
Font_Title   = 20;
Tick         = 5;
Tick_BOX     = 12;
L_Wid        = 3;

omega_x = Xout_Quat(5, :);
omega_y = Xout_Quat(6, :);
omega_z = Xout_Quat(7, :);

omega_x_ref = Xdout_Quat(1, :);
omega_y_ref = Xdout_Quat(2, :);
omega_z_ref = Xdout_Quat(3, :);

figure('Name','Attitude Tracking Error','NumberTitle','off');
plot(Tout_Quat, norms_Quat, 'b-', 'LineWidth', L_Wid);
grid on;
xlabel('Time (sec)', 'FontSize', Font_x, 'Interpreter', 'latex');
ylabel('Normalized Euclidean Distance', 'FontSize', Font_y, 'Interpreter', 'latex');
title('Attitude Tracking Error vs Time', 'FontSize', Font_Title, 'Interpreter', 'latex');

figure('Name','Angular Velocities','NumberTitle','off');

subplot(3, 1, 1)
plot(Tout_Quat, omega_x, 'b-', 'LineWidth', L_Wid)
hold on
plot(Tout_Quat, omega_x_ref, 'r--', 'LineWidth', L_Wid)
xlabel('Time (sec)', 'FontSize', Font_x, 'Interpreter', 'latex')
ylabel('$\Omega_x$', 'FontSize', Font_y, 'Interpreter', 'latex')
legend({'$\Omega_x$','$\Omega_{x,ref}$'}, 'FontSize', Font_Legend, 'Interpreter', 'latex')
title('Angular velocity $\Omega_x$', 'FontSize', Font_Title, 'Interpreter', 'latex')
grid on

subplot(3, 1, 2)
plot(Tout_Quat, omega_y, 'b-', 'LineWidth', L_Wid)
hold on
plot(Tout_Quat, omega_y_ref, 'r--', 'LineWidth', L_Wid)
xlabel('Time (sec)', 'FontSize', Font_x, 'Interpreter', 'latex')
ylabel('$\Omega_y$', 'FontSize', Font_y, 'Interpreter', 'latex')
legend({'$\Omega_y$','$\Omega_{y,ref}$'}, 'FontSize', Font_Legend, 'Interpreter', 'latex')
title('Angular velocity $\Omega_y$', 'FontSize', Font_Title, 'Interpreter', 'latex')
grid on

subplot(3, 1, 3)
plot(Tout_Quat, omega_z, 'b-', 'LineWidth', L_Wid)
hold on
plot(Tout_Quat, omega_z_ref, 'r--', 'LineWidth', L_Wid)
xlabel('Time (sec)', 'FontSize', Font_x, 'Interpreter', 'latex')
ylabel('$\Omega_z$', 'FontSize', Font_y, 'Interpreter', 'latex')
legend({'$\Omega_z$','$\Omega_{z,ref}$'}, 'FontSize', Font_Legend, 'Interpreter', 'latex')
title('Angular velocity $\Omega_z$', 'FontSize', Font_Title, 'Interpreter', 'latex')
grid on

%% Extract body torque components
Mx = uout_Quat(1, :);
My = uout_Quat(2, :);
Mz = uout_Quat(3, :);


figure('Name','Body Torques','NumberTitle','off');

subplot(3, 1, 1)
plot(Tout_Quat, Mx, 'm-', 'LineWidth', L_Wid)
xlabel('Time (sec)', 'FontSize', Font_x, 'Interpreter', 'latex')
ylabel('$M_x$', 'FontSize', Font_y, 'Interpreter', 'latex')
title('Body Torque $M_x$', 'FontSize', Font_Title, 'Interpreter', 'latex')
grid on

subplot(3, 1, 2)
plot(Tout_Quat, My, 'g-', 'LineWidth', L_Wid)
xlabel('Time (sec)', 'FontSize', Font_x, 'Interpreter', 'latex')
ylabel('$M_y$', 'FontSize', Font_y, 'Interpreter', 'latex')
title('Body Torque $M_y$', 'FontSize', Font_Title, 'Interpreter', 'latex')
grid on

subplot(3, 1, 3)
plot(Tout_Quat, Mz, 'b-', 'LineWidth', L_Wid)
xlabel('Time (sec)', 'FontSize', Font_x, 'Interpreter', 'latex')
ylabel('$M_z$', 'FontSize', Font_y, 'Interpreter', 'latex')
title('Body Torque $M_z$', 'FontSize', Font_Title, 'Interpreter', 'latex')
grid on
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%        





