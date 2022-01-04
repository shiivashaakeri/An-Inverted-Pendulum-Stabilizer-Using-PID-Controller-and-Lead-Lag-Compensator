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

num1 = [m*l/q  0 ];
den1 = [1  b*(J+m*l^2)/q  -(M+m)*m*g*l/q  -b*m*g*l/q  ];
G_pend=tf(num1,den1);
C = 10*((0.91*s + 1)*(0.2*s + 1))/s;
T = C*G_pend;
P = T / (T+1);
impulse(P)