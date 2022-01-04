clc
clear
close all

s = tf('s');


G2_pend = 4.54 / (s^3 + 0.18* (s^2) - 31.18*s - 4.45);
rlocus(G2_pend);