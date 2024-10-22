close all
clear
clc
syms s;

M = 0.5;  % arm mass
m = 0.2;  % The mass of the pendulum
l = 0.3;  % The length of the base of the pendulum
I = 0.006;  % The moment of inertia of the arm
g = 9.81;  % Downward acceleration

% Define the matrix A symbolically
A = sym([0, 1, 0, 0;
        0, 0, -((M+m)*g*l)/(M+m-m*l^2), 0;
        0, 0, 0, 1;
        0, 0, ((M+m)*g)/(I+m*l^2-M*m*l^2), 0]);

% B Define the matrix B symbolically
B = sym([0; (m*l)/(M+m-m*l^2); 0; -m*l/(I+m*l^2-M*m*l^2)]);

% Define matrix C and D for output s*theta
C = sym([1, 0, 0, 0]);
D = sym([0]);

% Function to convert to a fraction
G_sym = simplify(C*inv(s*eye(4)-A)*B+D)


