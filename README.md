# RotoTransform-Toolbox

## Overview

**RotoTransform-Toolbox** is a MATLAB library for transforming and manipulating rotation representations in 3D space. It allows conversions between Euler angles, quaternions, Rodriguez parameters, and the special orthogonal group \( SO(3) \). 

## Features

- **Some Transformations**:
  - `eulerToSO3.m`: Convert Euler angles to \( SO(3) \).
  - `quaternionMultip...`: Perform quaternion multiplication.
  - `rodriguezToS03.m`: Convert Rodriguez parameters to \( SO(3) \).
  - `SO3ToEuler.m`: Convert \( SO(3) \) to Euler angles.
  
- **Derivatives**:
  - `J_euler.m`: Calculate the Jacobian of Euler angles.
  
- **Utilities**:
  - `vex.m`: Compute the vector exponential.

## Examples

The toolbox includes two example simulations to demonstrate its capabilities. Check the `examples` folder for practical usage of the transformations.

## Installation

Clone the repository and add it to your MATLAB path:

```bash
git clone https://github.com/yourusername/RotoTransform-Toolbox.git
addpath('path_to_RotoTransform-Toolbox');
