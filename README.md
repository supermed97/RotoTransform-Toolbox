# RotoTransform-Toolbox

## Overview

**RotoTransform-Toolbox** is a MATLAB library for transforming and manipulating rotation representations in 3D space. It allows conversions between Euler angles, quaternions, Rodriguez parameters, and the special orthogonal group \( SO(3) \). 

## Features

- **Some Transformations Among The Many **:
  - `eulerToSO3(phi, theta, psi)`: Convert Euler angles to \( SO(3) \).
  - `rodriguezToSO3(rho)`: Convert Rodriguez parameters to \( SO(3) \).
  - `SO3ToEuler(R)`: Convert \( SO(3) \) to Euler angles.

- **Derivatives**:
  - `J_euler(phi, theta)`: Calculate the Jacobian of Euler angles.

- **Utilities**:
  - `skewSymmetric(v)`: Map a vector from \( \mathbb{R}^3 \) to \( so(3) \).
  - `vex(A)`: Convert a skew-symmetric matrix \( [\cdot]_\times \) to vector form.

## Examples

The toolbox includes two example simulations to demonstrate its capabilities. Check the `examples` folder for practical usage of the transformations.

## Installation

Clone the repository and add it to your MATLAB path:

```bash
git clone https://github.com/yourusername/RotoTransform-Toolbox.git
addpath('path_to_RotoTransform-Toolbox');
