close all
clear
clc
% Define variable s as Laplace variable
syms s;

% Definition of matrix A, B, C and D
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

% Function to convert to a fraction
G = simplify(C*inv(s*eye(4)-A)*B+D)

% Another method 
[num,den] = ss2tf(A,B,C,D)
printsys(num,den)


% Calculate the response of the system to the step input

F=tf(num,den);
t = 0:0.01:2;
figure()
y1 = step(F,t);
plot(t,y1);


% Calculate the response of the system to the shock input 
figure()
y2=impulse(F,t); 
plot(t,y2)

% Calculation of system response to sinusoidal input
figure()
u=sin(10*t);              
y3=lsim(F,u,t);
plot(t,y3)

% Obtaining system poles:
pole(F)

% finding the zeros of the system
zero(F)

rlocus(F)




