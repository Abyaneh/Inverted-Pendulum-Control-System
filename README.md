# Inverted Pendulum Control System

![Project Image_1](https://github.com/Abyaneh/Inverted-Pendulum-Control-System/blob/main/Code%20and%20photos/picture/1.jpg) 

![Project Image_2](https://github.com/Abyaneh/Inverted-Pendulum-Control-System/blob/main/Code%20and%20photos/picture/shape.jpg) 

## Table of Contents
- [Introduction](#introduction)
- [Features](#features)
- [System Dynamics](#system-dynamics)
- [State-Space Model and Transfer Functions](#state-space-model-and-transfer-functions)
- [Matlab Code for Free Body Diagram](#matlab-code-for-free-body-diagram)
- [Technologies & Tools Used](#technologies--tools-used)
- [How to Run the Project](#how-to-run-the-project)
- [Contributing](#contributing)
- [License](#license)

## Introduction
The inverted pendulum is a classic problem in dynamics and control theory, where the goal is to design a controller to keep a pendulum upright by moving a base. This project involves modeling, simulation, and controller design to stabilize the system using Matlab and other tools. The pendulum starts at an unstable equilibrium (θ = 0) and needs to be balanced at the top (θ = 180°).

## Features
- **Non-linear system control**: Stabilizing the pendulum at a 180° angle.
- **Dynamic modeling**: Derived system equations with state-space representation.
- **Visualization**: Free-body diagrams, response plots, and system behavior simulations in Matlab.
- **Controller Design**: PID controller implementation for pendulum stability.

## System Dynamics
The system is modeled as a two-degree-of-freedom problem where the arm's motion and the pendulum's motion must be controlled. The mathematical model is represented by the following equations:
- Dynamic equation: `Ml²θ̈ + Bl²θ̇ + Mgl²sin(θ) = Ml²u`
- State-space representation:
  ```matlab
  A = [0 1 0 0;
       0 0 -((M+m)*g*l)/(M+m-m*l²) 0;
       0 0 0 1;
       0 0 ((M+m)*g)/(I+m*l²-M*m*l²) 0];
  ```

## State-Space Model and Transfer Functions
The state-space model is derived and transformed into transfer functions using Laplace transformations. The key steps include:
- Linearization of dynamic equations.
- Derivation of transfer functions using Matlab.

The resulting transfer function is:
```matlab
G(s) = 0.08798/s³ + 2s/s² - 458.035401
```

## Matlab Code for Free Body Diagram
Below is an example of Matlab code used to draw a free-body diagram and solve for forces acting on the system:
```matlab
% Matlab code for Free Body Diagram
close all;
clear;
clc;
x = 2;
y = 3;
z = 1;
F = 10;
g = 9.81;

Rx = F;
Ry = 0;
Rz = 0 - g;

figure;
quiver3(x, y, z, Rx, Ry, Rz, 'LineWidth', 2);
xlabel('X');
ylabel('Y');
zlabel('Z');
title('Free Body Diagram of a Body in Space');
grid on;
```

## Technologies & Tools Used
- **Matlab/Simulink**: For dynamic modeling, simulations, and controller design.
- **Control Theory**: Applied to analyze and stabilize the inverted pendulum system.
- **State-Space Modeling**: Used to represent the system and derive transfer functions.
  
## How to Run the Project
1. Clone the repository:
   ```bash
   git clone https://github.com/Abyaneh/Inverted-Pendulum-Control-System/tree/main
   cd inverted-pendulum-system
   ```
2. Open Matlab and navigate to the project directory.
3. Run the main script `pendulum_control.m` to simulate and visualize the system's response.


## Contributing
Contributions are welcome! Please follow these steps:
1. Fork the repository.
2. Create your feature branch (`git checkout -b feature/AmazingFeature`).
3. Commit your changes (`git commit -m 'Add some AmazingFeature'`).
4. Push to the branch (`git push origin feature/AmazingFeature`).
5. Open a pull request.

## License
This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

