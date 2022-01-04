clc
clear
close all

s = tf('s');
M = 0.5;
m = 0.2;
b = 0.1;
J = 0.006;
g = 9.8;
l = 0.3;
q = (M + m) * (J + m * l^2)-(m * l)^2;

num1 = [m*l/q  0  0];
den1 = [1  b*(J+m*l^2)/q  -(M+m)*m*g*l/q  -b*m*g*l/q  0];
G_pend=tf(num1,den1);
controlSystemDesigner('bode', G_pend*(1/s));