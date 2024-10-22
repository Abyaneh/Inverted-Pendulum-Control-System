close all
clear
clc
% Definition of body coordinates
x = 2;
y = 3;
z = 1;

% Definition of incoming force
F = 10;
Fx = F;
Fy = 0;
Fz = 0;

% Definition of force position
nx = 1;
ny = 0;
nz = 0;

% Definition of gravity
g = 9.81;

% Calculate the result vector
Rx = Fx;
Ry = Fy;
Rz = Fz - g;

% Draw a free body diagram
figure
quiver3(x, y, z, Rx, Ry, Rz, 'LineWidth', 2)
xlabel('X')
ylabel('Y')
zlabel('Z')
title('Free Body Diagram of a Body in Space')
grid on