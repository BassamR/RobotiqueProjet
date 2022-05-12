close all

realAngle = [0; 10; 20; 30; 40; 50; 60; 70; 80; 90; 100; 110; 120; 130; 140; 150; 160; 170; 180];
measuredAngle = [55.59; 14; 33; 51.68; 57.59; 63.04; 67.72; 71; 85; 92; 97; 109.59; 114; 124; 131.59; 133.63; 141; 110; 125];

plot(realAngle, measuredAngle, '*', 'MarkerSize', 8)
title('Angle réel en fonction de l''angle calculé')

hold on

syms x;
f=x;
fplot(x, f, [0,200], 'r')

hold off

xlabel('Angle Réel [deg]');
ylabel('Angle Calculé [deg]');