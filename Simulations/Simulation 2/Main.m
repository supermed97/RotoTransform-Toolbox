%%

clear all
close all
clc
GenerateHandlers2;  % creates function handler for omega vector

global Ts  RunT  

%%%%% Intermittency %%%%%% 
Ts = 0.01;   % Sampling time  (jump between Tm and TM)
Tm = Ts*1;   % T min interval
TM = Ts*1;   % T max interval 

% Run time
N = 30/Ts;
RunT = Tm + (TM-Tm).*rand(N,1);


% Intial conditions
 R_0 = [
    0.6679, -0.1808, 0.7219;
    0.6552,  0.6030, -0.4551;
   -0.3530,  0.7770,  0.5213];

    x0_Euler=SO3ToEuler(R_0);
    x0_Rod=SO3ToRodriguez(R_0);
    x0_Quat=so3ToQuaternion(R_0);

  [Tout_Quat,Error_Quat,Xout_Quat,Xdout_Quat,uout_Quat] = QuatOdeIntegrator(x0_Quat,OMEGA_function);
  [Tout_Euler,Error_Euler,Xout_Euler,Xdout_Euler,uout_Euler,norms_Euler] = EulerOdeIntegrator(x0_Euler,OMEGA_function);
  [Tout_Rod,Error_Rod,Xout_Rod,Xdout_Rod,uout_Rod] = RodriguezOdeIntegrator(x0_Rod,OMEGA_function);
  [Tout_SO3,Error_SO3,Xout_SO3,Xdout_SO3,uout_SO3,norms_SO3] = SO3OdeIntegrator(R_0,OMEGA_function);


%% Plots

T_quat  = Tout_Quat;
T_euler = Tout_Euler;
T_Rod =  Tout_Rod;

%% Adding values to the plot axis (Percent)
LIMIT        = 0.03;

Font_x       = 20;
Font_y       = 20;
Font_Legend  = 20;
Font_Title   = 20;
Tick         = 5;
Tick_BOX     = 12;
L_Wid        = 3;

%% Data from Workspace  TO PLOT
    q0(1,:)         = Xout_Quat(1,:); %q0
    q1(1,:)         = Xout_Quat(2,:); %q1
    q2(1,:)         = Xout_Quat(3,:);  %q2
    q3(1,:)         = Xout_Quat(4,:);  %q3

    %% Rodriguez

    p1(1,:)         = Xout_Rod(1,:); %p1
    p2(1,:)         = Xout_Rod(2,:); %p2
    p3(1,:)         = Xout_Rod(3,:);  %p3
%%  Euler angles converted to degrees
    % phi(1,:)         = Xout_Euler(1,:); %phi
    % theta(1,:)       = Xout_Euler(2,:); %theta
    % psi(1,:)         = Xout_Euler(3,:);  %psi


    phi(1,:)         = rad2deg(Xout_Euler(1,:)); %phi
    theta(1,:)       = rad2deg(Xout_Euler(2,:)); %theta
    psi(1,:)         = rad2deg(Xout_Euler(3,:));  %psi





%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%        
%%  Euler plots  
    figure(1)
subplot(1,3,1)
    plot(T_euler, phi, 'm-', 'linewidth', L_Wid)
    hold on
    xlabel('Time (sec)', 'FontSize', Font_x, 'Interpreter', 'latex')
    ylabel('$\phi$', 'FontSize', Font_y, 'Interpreter', 'latex')
    legend({'$\phi$'}, 'FontSize', Font_Legend, 'Interpreter', 'latex')
    title('Euler $\phi$', 'FontSize', Font_Title, 'Interpreter', 'latex')
    grid on

subplot(1,3,2)
    plot(T_euler, theta, 'm-', 'linewidth', L_Wid)
    hold on
    xlabel('Time (sec)', 'FontSize', Font_x, 'Interpreter', 'latex')
    ylabel('$\theta$', 'FontSize', Font_y, 'Interpreter', 'latex')
    legend({'$\theta$'}, 'FontSize', Font_Legend, 'Interpreter', 'latex')
    title('Euler $\theta$', 'FontSize', Font_Title, 'Interpreter', 'latex')
    grid on

    subplot(1,3,3)
    plot(T_euler, psi, 'm-', 'linewidth', L_Wid)
    hold on
    xlabel('Time (sec)', 'FontSize', Font_x, 'Interpreter', 'latex')
    ylabel('$\psi$', 'FontSize', Font_y, 'Interpreter', 'latex')
    legend({'$\psi$'}, 'FontSize', Font_Legend, 'Interpreter', 'latex')
    title('Euler $\psi$', 'FontSize', Font_Title, 'Interpreter', 'latex')
    grid on
%% Rodriguez plots
figure(2)
subplot(1,3,1)
    plot(T_Rod, p1, 'r-', 'linewidth', L_Wid)
    hold on
    xlabel('Time (sec)', 'FontSize', Font_x, 'Interpreter', 'latex')
    ylabel('$\rho_1$', 'FontSize', Font_y, 'Interpreter', 'latex')
    legend({'$\rho_1$'}, 'FontSize', Font_Legend, 'Interpreter', 'latex')
    title('Rodriguez $\rho_1$', 'FontSize', Font_Title, 'Interpreter', 'latex')
    grid on

subplot(1,3,2)
    plot(T_Rod, p2, 'r-', 'linewidth', L_Wid)
    hold on
    xlabel('Time (sec)', 'FontSize', Font_x, 'Interpreter', 'latex')
    ylabel('$\rho_2$', 'FontSize', Font_y, 'Interpreter', 'latex')
    legend({'$\rho_2$'}, 'FontSize', Font_Legend, 'Interpreter', 'latex')
    title('Rodriguez $\rho_2$', 'FontSize', Font_Title, 'Interpreter', 'latex')
    grid on

    subplot(1,3,3)
    plot(T_Rod, p3, 'r-', 'linewidth', L_Wid)
    hold on
    xlabel('Time (sec)', 'FontSize', Font_x, 'Interpreter', 'latex')
    ylabel('$\rho_3$', 'FontSize', Font_y, 'Interpreter', 'latex')
    legend({'$\rho_3$'}, 'FontSize', Font_Legend, 'Interpreter', 'latex')
    title('Rodriguez $\rho_3$', 'FontSize', Font_Title, 'Interpreter', 'latex')
    grid on
%% Quaternion plots
figure(3)
subplot(2, 2, 1)
    plot(T_quat, q0, 'b-', 'linewidth', L_Wid)
    hold on
    xlabel('Time (sec)', 'FontSize', Font_x, 'Interpreter', 'latex')
    ylabel('$q_0$', 'FontSize', Font_y, 'Interpreter', 'latex')
    legend({'$q_0$'}, 'FontSize', Font_Legend, 'Interpreter', 'latex')
    title('Quaternion $q_0$', 'FontSize', Font_Title, 'Interpreter', 'latex')
    grid on

subplot(2, 2, 2)
    plot(T_quat, q1, 'b-', 'linewidth', L_Wid)
    hold on
    xlabel('Time (sec)', 'FontSize', Font_x, 'Interpreter', 'latex')
    ylabel('$q_1$', 'FontSize', Font_y, 'Interpreter', 'latex')
    legend({'$q_1$'}, 'FontSize', Font_Legend, 'Interpreter', 'latex')
    title('Quaternion $q_1$', 'FontSize', Font_Title, 'Interpreter', 'latex')
    grid on

subplot(2, 2, 3)
    plot(T_quat, q2, 'b-', 'linewidth', L_Wid)
    hold on
    xlabel('Time (sec)', 'FontSize', Font_x, 'Interpreter', 'latex')
    ylabel('$q_2$', 'FontSize', Font_y, 'Interpreter', 'latex')
    legend({'$q_2$'}, 'FontSize', Font_Legend, 'Interpreter', 'latex')
    title('Quaternion $q_2$', 'FontSize', Font_Title, 'Interpreter', 'latex')
    grid on

subplot(2, 2, 4)
    plot(T_quat, q3, 'b-', 'linewidth', L_Wid)
    hold on
    xlabel('Time (sec)', 'FontSize', Font_x, 'Interpreter', 'latex')
    ylabel('$q_3$', 'FontSize', Font_y, 'Interpreter', 'latex')
    legend({'$q_3$'}, 'FontSize', Font_Legend, 'Interpreter', 'latex')
    title('Quaternion $q_3$', 'FontSize', Font_Title, 'Interpreter', 'latex')
    grid on
%%  error in norms
% Ensure norms_Euler has the correct dimensions
norms_Euler = norms_Euler';  % Transpose to make it 3000x1 if needed

% Calculate the error
error_norms = norms_SO3 - norms_Euler;

% Plot the error on Figure 4 with LaTeX-styled labels
figure(4);
plot(T_euler,error_norms,'r-', 'linewidth', L_Wid);
xlabel('Time','FontSize', Font_x, 'Interpreter', 'latex');
ylabel('$\|R\|_I - \|R_E\|_I$','FontSize', Font_Legend, 'Interpreter', 'latex');
title('Error in Norms: $\|R\|_I - \|R_E\|_I$', 'Interpreter', 'latex');
grid on;


