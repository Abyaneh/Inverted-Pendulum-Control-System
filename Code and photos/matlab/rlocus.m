close all
clear
clc
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

sys = ss(A,B,C,D);

% conversion function
[num,den] = ss2tf(A,B,C,D)

printsys(num,den)
f=tf(num,den)
%figure(1)

%step(f)
figure(2)
rlocus(f)

GS3 = 2*s/(s^2-458.035401)
 num4 = [2,0];
 den4 = [1,0,-458.035401]; 
 f4 = tf(num4,den4)


kp = 246.229;
ki = 32308.4671;
kd = 0.042127;


c = kp + ki*(1/s) + kd*s;

GS_final = c*GS3
num5 = [9.375,24.225,8.7156];
den5 = [1,-457.29,-455.38];
f5 = tf(num5,den5)
%figure(3)
% step(f5)

 figure(4)
 rlocus(f5)
