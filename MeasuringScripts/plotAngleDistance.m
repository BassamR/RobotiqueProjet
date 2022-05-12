close all

distanceFromRobot = [0] %cm
measuredAngle = [];

plot(realAngle, measuredAngle, '*', 'MarkerSize', 8)
title('Angle réel en fonction de l''angle calculé')

hold on

syms x;
f=x;
fplot(x, f, [0,200], 'r')

hold off

xlabel('Angle Réel [deg]');
ylabel('Angle Calculé [deg]');