close all
clear
clc
syms s;

% تعریف ماتریس A، B، C و D
M = 0.5; % جرم آرم
m = 0.2; % جرم پاندول
l = 0.3; % طول پایه پاندول
I = 0.006; % مقدار ممان اینرسی آرم
g = 9.81; % شتاب نزولی

A = [0, 1, 0, 0;
     0, 0, -((M+m)*g*l)/(M+m-m*l^2), 0;
     0, 0, 0, 1;
     0, 0, ((M+m)*g)/(I+m*l^2-M*m*l^2), 0];
 
B = [0; (m*l)/(M+m-m*l^2); 0; -m*l/(I+m*l^2-M*m*l^2)];

C = [1, 0, 0, 0];
D = [0];

sys = ss(A,B,C,D);

%تابع تبدیل 
[num,den] = ss2tf(A,B,C,D)

printsys(num,den)
f=tf(num,den)
figure(1)

step(f)




%%%%%%%%%%%%


% G(s) = 0.08798/s^3 + 2s/s^2-458.035401
%-42.7094

GS2 = -42.7094/(s^2-458.035401)
 num3 = [-42.7094];
 den3 = [1,0,-458.035401];
 f3 = tf(num3,den3)
figure(2)
 step(f3)
title("Approximate step Response1 ")


%%%%%%%%%

GS3 = 2*s/(s^2-458.035401)
 num4 = [2,0];
 den4 = [1,0,-458.035401]; 
 f4 = tf(num4,den4)
figure(3)
 step(f4)
title("Approximate step Response2 ")



kp = 246.229;
ki = 32308.4671;
kd = 0.042127;


c = kp + ki*(1/s) + kd*s;

GS_final = c*GS3
num5 = [9.375,24.225,8.7156];
den5 = [1,-457.29,-455.38];
f5 = tf(num5,den5)
figure(4)
 step(f5)

%step(GS_final)
