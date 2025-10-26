# 
This project implements and simulates **rigid-body attitude dynamics and control** on the **special orthogonal group** SO(3), following the geometric control framework presented in:

> **Taeyoung Lee, Melvin Leok, and N. Harris McClamroch**  
> *"Geometric Tracking Control of a Quadrotor UAV on SE(3)"*,  
> IEEE Conference on Decision and Control (CDC), 2010.  

---

## 

The simulation models a rigid body with inertia matrix \( J \) and angular velocity \( \Omega \), evolving on SO(3):

\[
\dot{R} = R[\Omega]_\times, \qquad
J\dot{\Omega} + \Omega \times J\Omega = M
\]

We compute the body torque \( M \) to track a desired attitude and angular velocity trajectory.


## ⚙️ Main Components

### `main.m`
- Defines parameters, control gains, and inertia \( J \)
- Initializes the initial rotation \( R_0 \) and angular velocity \( \Omega_0 \)
- Calls the `SO3OdeIntegrator` to simulate dynamics
- Plots:
  - Attitude tracking error (Normalized Euclidean distance)
  - Control torques over time

### `SO3OdeIntegrator.m`
- Integrates the rigid body dynamics on SO(3) using MATLAB's `ode45`
- Evolves both rotation \( R(t) \) and angular velocity \( \Omega(t) \)
- Returns simulation histories of:
  - Rotation matrices
  - Angular velocities
  - Control torques
  - Attitude error norms

### `bodyTorque.m`
Implements the control law:
\[
M = -k_R e_R - k_\Omega e_\Omega + \Omega \times J\Omega
    - J\left([\Omega]_\times R_e^T\Omega_d - R_e^T\dot{\Omega}_d\right)
\]

where:
\[
R_e = R_d^T R, \quad
e_R = \tfrac{1}{2} \, vex(R_e - R_e^T), \quad
e_\Omega = \Omega - R_e^T \Omega_d
\]

### Supporting Files
- **`skewSymmetricMatrix.m`** — returns the skew-symmetric matrix \( [a]_\times \)
- **`unpack.m`** — extracts \( R, \Omega, R_d \) from the state vector
- **`get_desired_angular_velocity.m`** — defines \( \Omega_d(t) \) and \( \dot{\Omega}_d(t) \)
- **`eulerToSO3.m`** — converts Euler angles (ϕ, θ, ψ) to a rotation matrix

---

## 

The simulation produces:
- **Attitude error norm** vs. time  
- **Angular velocity error** vs. time  
- **Control torques** vs. time  
- Optionally, rotation trajectories or orientation visualizations

---

## 

```matlab
phi = pi/3; theta = -pi/4; psi = pi/6;
R_0 = eulerToSO3(phi, theta, psi);
omega0 = zeros(3,1);
Rd0 = eye(3);

x0 = [R_0(:); omega0; Rd0(:)];
[Tout_SO3, Error_SO3, Xout_SO3, Xdout_SO3, uout_SO3, norms_SO3] = SO3OdeIntegrator(x0, J);


