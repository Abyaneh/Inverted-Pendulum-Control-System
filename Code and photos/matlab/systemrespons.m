close all
clear
clc
% تعریف متغیر s به عنوان متغیر Laplace
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

% تابع تبدیل به کسری
G = simplify(C*inv(s*eye(4)-A)*B+D)

%روش دیگر 
[num,den] = ss2tf(A,B,C,D)
printsys(num,den)


% محاسبه پاسخ سیستم به ورودی پله

F=tf(num,den);
t = 0:0.01:2;
figure()
y1 = step(F,t);
plot(t,y1);


% محاسبه پاسخ سیستم به ورودی ضربه 
figure()
y2=impulse(F,t); 
plot(t,y2)

%محاسبه پاسخ سیستم به ورودی سینوسی
figure()
u=sin(10*t);              
y3=lsim(F,u,t);
plot(t,y3)

% بدست آوردن قطب‌هایسیستم:
pole(F)

% بدست آوردن صفرهای سیستم
zero(F)

rlocus(F)




