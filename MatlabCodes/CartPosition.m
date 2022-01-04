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

Kp = 100;
Ki = 1;
Kd = 20;
C = pid(Kp,Ki,Kd);

num1 = [m*l/q  0 ];
den1 = [1  b*(J+m*l^2)/q  -(M+m)*m*g*l/q  -b*m*g*l/q  ];
G_pend=tf(num1,den1);
G_cart = (((J+m*l^2)/q)*s^2 - (m*g*l/q))/(s^4 + (b*(J + m*l^2))*s^3/q - ((M + m)*m*g*l)*s^2/q - b*m*g*l*s/q);
T2 = feedback(1,G_pend*C)*G_cart;
t = 0:0.01:5;
impulse(T2, t);