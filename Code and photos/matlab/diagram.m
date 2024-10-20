close all
clear
clc
% تعریف مختصات جسم
x = 2;
y = 3;
z = 1;

% تعریف نیروی وارده
F = 10;
Fx = F;
Fy = 0;
Fz = 0;

% تعریف موقعیت نیرو
nx = 1;
ny = 0;
nz = 0;

% تعریف شدت گرانش
g = 9.81;

% محاسبه بردار نتیجه
Rx = Fx;
Ry = Fy;
Rz = Fz - g;

% رسم دیاگرام جسم آزاد
figure
quiver3(x, y, z, Rx, Ry, Rz, 'LineWidth', 2)
xlabel('X')
ylabel('Y')
zlabel('Z')
title('Free Body Diagram of a Body in Space')
grid on