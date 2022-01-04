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

G_pend = (m*l*s/q)/(s^3 + (b*(J + m*l^2))*s^2/q - ((M + m)*m*g*l)*s/q - b*m*g*l/q);
G_cart = (((J+m*l^2)/q)*s^2 - (m*g*l/q))/(s^4 + (b*(J + m*l^2))*s^3/q - ((M + m)*m*g*l)*s^2/q - b*m*g*l*s/q);

sys_tf = [G_cart ; G_pend];

inputs = {'u'};
outputs = {'x'; 'phi'};

set(sys_tf,'InputName',inputs)
set(sys_tf,'OutputName',outputs)

sys_tf