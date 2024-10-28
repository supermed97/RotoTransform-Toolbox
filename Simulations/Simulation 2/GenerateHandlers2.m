% Define the function handle for omega(t)
OMEGA_function = @(t) [0.3 * sin(0.8422 * t);
              0.21 * sin(0.3682 * t + pi);
              0.15 * sin(1.4516 * t + pi/3)];
