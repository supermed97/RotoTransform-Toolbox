% Define the function handle for omega(t)
OMEGA_function = @(t) [0.1 * sin(0.3376 * t);
              0.07 * sin(0.6079 * t + pi);
              0.05 * sin(0.7413 * t + pi/3)];
