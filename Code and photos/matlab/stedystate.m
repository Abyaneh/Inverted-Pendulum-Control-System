close all
clear
clc
M = 0.5;  % arm mass
m = 0.2;  % The mass of the pendulum
l = 0.3;  % The length of the base of the pendulum
I = 0.006;  % The moment of inertia of the arm
g = 9.81;  % Downward acceleration

A = [0, 1, 0, 0;
     0, 0, -((M+m)*g*l)/(M+m-m*l^2), 0;
     0, 0, 0, 1;
     0, 0, ((M+m)*g)/(I+m*l^2-M*m*l^2), 0];
 
B = [0; (m*l)/(M+m-m*l^2); 0; -m*l/(I+m*l^2-M*m*l^2)];

C = [1, 0, 0, 0];
D = [0];

sys = ss(A, B, C, D)