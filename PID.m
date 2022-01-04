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
Kp = 100;
Ki = 1;
Kd = 20;
C = pid(Kp,Ki,Kd);
T = feedback(G_pend,C);
t=0:0.01:10;
impulse(T,t)
axis([0, 2.5, -0.2, 0.2]);
