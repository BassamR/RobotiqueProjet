ref_distance= 20; %cm
%% speed measured versus actual speed with 20mm sensitivity and reference at 20cm
real_speed= ones(16,1);
for i = 1:16
    real_speed(i)=(i-1)*62*13/1000; % in cm/s
end
measured_speed= [0 0.7297 1.5273 2.3556 3.17146 3.9272 4.8515 5.154 6.3437 6.8739 8.2471 9.1636 10.3062 10.30776 11.7819 11.7851]; %in cm/s
measured_speed=measured_speed';
error_speed= ones(size(measured_speed));
error_speed=abs(real_speed-measured_speed);
figure("name",'real speed versus measured speed')
hold on
grid on
plot(real_speed,error_speed,'^-r')
plot (real_speed,measured_speed,"b.-")
title('measured speed versus actual speed')
legend('speed error', "measured speed")
xlabel('actual speed of epuck [cm/s]') 
ylabel('speed measured [cm/s]') 
hold off
%% speed measured versus distance from ToF all with 20mm sensitivity

speed_low = 3.991; %cm/s
speed_middle =5.2;%cm/s
speed_high =8.06;%cm/s

distance_vector=ones(6,1);

for i = 1:6
    distance_vector(i)=(i-1)*2+5; % in cm
end

measured_speed_low = [3.748336 3.7488365 3.748016357 3.926590204 4.580941677 4.850792884 ];
measured_speed_low=measured_speed_low';
error_speed_low=abs(measured_speed_low-speed_low*ones(6,1));

measured_speed_middle =[4.85213 4.582438 4.581997 5.15336 5.49738 5.8905 ];
measured_speed_middle=measured_speed_middle';
error_speed_middle=abs(measured_speed_middle-speed_middle*ones(6,1));

measured_speed_high=[7.496716 7.4963553 7.497166 8.24517 8.245604 10.3070];
measured_speed_high=measured_speed_high';
error_speed_high=abs(measured_speed_high-speed_high*ones(6,1));

figure("name",'measured speed versus distance')
hold on
grid on
plot(distance_vector,measured_speed_low,'.-r')
plot(distance_vector,measured_speed_middle,'.-g')
plot(distance_vector,measured_speed_high,'.-b')

plot(distance_vector,error_speed_low,'.--r')
plot(distance_vector,error_speed_middle,'.--g')
plot(distance_vector,error_speed_high,'.--b')


title('measured speed versus distance from epuck')
legend('actual epuck speed 4[cm/s]', "actual epuck speed 5.2 [cm/s]","actual epuck speed 8[cm/s]","measurment error at 4 [cm/s]","measurment error at 5.2 [cm/s]","measurment error at 8 [cm/s]")
%text(7,2,"error between measured and actual speed")
xlabel('distance from epuck[cm]') 
ylabel('measured speed [cm/s]') 
hold off
