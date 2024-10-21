# Inverted Pendulum Control System

![Project Image_1](https://github.com/Abyaneh/Inverted-Pendulum-Control-System/blob/main/Code%20and%20photos/picture/shape.jpg) 

![Project Image_2](https://github.com/Abyaneh/Inverted-Pendulum-Control-System/blob/main/Code%20and%20photos/picture/1.jpg) 


## Table of Contents
- [Introduction](#introduction)
- [Features](#features)
- [System Dynamics](#system-dynamics)
- [State-Space Model and Transfer Functions](#state-space-model-and-transfer-functions)
- [Matlab Code for Free Body Diagram](#matlab-code-for-free-body-diagram)
- [PID Controller and Simulation Results](#pid-controller-and-simulation-results)
- [Technologies & Tools Used](#technologies--tools-used)
- [How to Run the Project](#how-to-run-the-project)
- [Contributing](#contributing)
- [License](#license)
- [Back to Top](#)

## Introduction
The inverted pendulum is a well-known problem in control theory where the goal is to stabilize a pendulum in an upright position. Using Matlab, this project involves modeling, simulating, and controlling the system with a PID controller to balance the pendulum at 180°, starting from an unstable position.

[Back to Top](#table-of-contents)

## Features
- Stabilization of a non-linear dynamic system using PID control.
- Free-body diagram and system response simulations.
- Control system designed and tested in Matlab.
- Visualizations and graphs of system behavior.

[Back to Top](#table-of-contents)

## System Dynamics
The inverted pendulum system has two degrees of freedom and is modeled using the following equations:
- Dynamic equation:  
  `Ml²θ̈ + Bl²θ̇ + Mgl²sin(θ) = Ml²u`
  
The goal is to keep the pendulum upright by applying appropriate control forces through the motor arm.

[Back to Top](#table-of-contents)

## State-Space Model and Transfer Functions
Linearization and the derivation of the state-space model result in the following matrix representation:
```matlab
A = [0 1 0 0;
     0 0 -((M+m)*g*l)/(M+m-m*l²) 0;
     0 0 0 1;
     0 0 ((M+m)*g)/(I+m*l²-M*m*l²) 0];
 
B = [0; (m*l)/(M+m-m*l²); 0; -m*l/(I+m*l²-M*m*l²)];
```

The transfer function derived for the system is:
```matlab
G(s) = 0.08798/s³ + 2s/s² - 458.035401
```

[Back to Top](#table-of-contents)

## Matlab Code for Free Body Diagram
The free-body diagram is an essential part of understanding the forces acting on the pendulum system. Below is an example Matlab script for generating the diagram:
```matlab
% Matlab code for Free Body Diagram
close all;
clear;
clc;

% Define object coordinates and forces
x = 2;
y = 3;
z = 1;
F = 10;
g = 9.81;

Rx = F;
Ry = 0;
Rz = 0 - g;

% Plot free-body diagram
figure;
quiver3(x, y, z, Rx, Ry, Rz, 'LineWidth', 2);
xlabel('X');
ylabel('Y');
zlabel('Z');
title('Free Body Diagram of a Body in Space');
grid on;
```

[Back to Top](#table-of-contents)

## PID Controller and Simulation Results

### PID Control Implementation
The PID controller is implemented to stabilize the pendulum at its upright position (θ = 180°). The controller parameters are tuned as follows:

```matlab
kp = 246.229;
ki = 32308.4671;
kd = 0.042127;

sys = pid(kp, ki, kd);
```

### Simulation Results
The following graphs show the system’s response to step input and the corresponding stabilization under the PID controller. The pendulum successfully stabilizes after initial oscillations:

1. **Step Response of the System:**

Step Response **before** PID control:

![Step Response Graph](https://github.com/Abyaneh/Inverted-Pendulum-Control-System/blob/main/Code%20and%20photos/picture/70_1.jpg) 

Step Response **after** PID control:

![Step Response Graph](https://github.com/Abyaneh/Inverted-Pendulum-Control-System/blob/main/Code%20and%20photos/picture/70_2.jpg) 

The graph above shows the system’s response to a step input with the PID controller applied, illustrating how quickly and smoothly the pendulum stabilizes.

2. **Impulse Response of the System:**
    ![Impulse Response Graph](#) <!-- Placeholder for Impulse Response Graph -->

    This graph demonstrates how the system reacts to a brief disturbance (impulse input), and how it returns to the stable equilibrium position under PID control.

4. **Root Locus Analysis:**

Root Locus Diagram **before** PID Control:

![Root Locus Diagram before PID Control](https://github.com/Abyaneh/Inverted-Pendulum-Control-System/blob/main/Code%20and%20photos/picture/76.jpg)

Root Locus Diagram **after** PID Control:

![Root Locus Diagram after PID Control](https://github.com/Abyaneh/Inverted-Pendulum-Control-System/blob/main/Code%20and%20photos/picture/77.jpg)
    
The root locus plot shows the placement of poles and zeros of the system both before and after applying the PID controller. The poles are moved toward more stable positions after applying the controller.

[Back to Top](#table-of-contents)

## Technologies & Tools Used
- **Matlab/Simulink**: Dynamic modeling, control design, and simulation.
- **Control Theory**: State-space analysis, PID controller design.
- **Visualization**: Free-body diagrams, step response, impulse response, and root locus plots.

[Back to Top](#table-of-contents)

## How to Run the Project
1. Clone the repository:
   ```bash
   git clone https://github.com/Abyaneh/Inverted-Pendulum-Control-System.git
   cd Inverted-Pendulum-Control-System
   ```
2. Open Matlab and run `pendulum_control.m` to simulate the system.
3. View the generated graphs and results for system behavior analysis.

[Back to Top](#table-of-contents)

## Contributing
Contributions are welcome! To contribute:
1. Fork the repository.
2. Create a new branch (`git checkout -b feature/YourFeature`).
3. Commit your changes (`git commit -m 'Add some feature'`).
4. Push to the branch (`git push origin feature/YourFeature`).
5. Open a pull request.

[Back to Top](#table-of-contents)

## License
This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for more information.

[Back to Top](#table-of-contents)

---

