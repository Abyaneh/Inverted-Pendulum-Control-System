close all
clear
clc
M = 0.5;  % جرم آرم
m = 0.2;  % جرم پاندول
l = 0.3;  % طول پایه پاندول
I = 0.006;  % مقدار ممان اینرسی آرم
g = 9.81;  % شتاب نزولی

A = [0, 1, 0, 0;
     0, 0, -((M+m)*g*l)/(M+m-m*l^2), 0;
     0, 0, 0, 1;
     0, 0, ((M+m)*g)/(I+m*l^2-M*m*l^2), 0];
 
B = [0; (m*l)/(M+m-m*l^2); 0; -m*l/(I+m*l^2-M*m*l^2)];

C = [1, 0, 0, 0];
D = [0];

sys = ss(A, B, C, D)