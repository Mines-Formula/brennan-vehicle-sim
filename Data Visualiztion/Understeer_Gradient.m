clear
clc
close all
addpath(genpath("Data Files"))

%data w/ rear o ARB
aySoft = readtable("AySoft.csv").ay*2;
steerSoft = readtable("SteerSoft.csv").steer/3.6+3;

%data w/ stiffest rear ARB
ayStiff = readtable("ayStiff.csv").ay*2;
steerStiff = readtable("steerStiff.csv").steer/3.6+3;

%% Eliminate NaN values
% Fill NaNs with last known value (forward fill)
aySoft     = fillmissing(aySoft, 'previous');
steerSoft  = fillmissing(steerSoft, 'previous');

ayStiff    = fillmissing(ayStiff, 'previous');
steerStiff = fillmissing(steerStiff, 'previous');

%% Smooth Data and define ellapsed time vectors
N_ay = 50;  % number of samples for running average for ay
N_steer = 100; % number of samples for running average steering

aySoft = movmean(aySoft, N_ay); 
steerSoft = movmean(steerSoft, N_steer); 
i_Soft = linspace(1,length(aySoft),length(aySoft));

ayStiff = movmean(ayStiff, N_ay); 
steerStiff = movmean(steerStiff, N_steer); 
i_Stiff = linspace(1,length(ayStiff),length(ayStiff));

%% Plot data vs index
figure;
plot(i_Soft, aySoft);
hold on
xlabel("Index")
ylabel("Lateral Acceleration (g)")
title("No Rear ARB")
grid on
hold off

figure;
plot(i_Soft, steerSoft);
hold on
xlabel("Index")
ylabel("Steering Angle (deg)")
title("No Rear ARB")
grid on
hold off

figure;
plot(i_Stiff, ayStiff);
hold on
xlabel("Index")
ylabel("Lateral Acceleration (g)")
title("Stiffest Rear ARB")
grid on
hold off

figure;
plot(i_Stiff, steerStiff);
hold on
xlabel("Index")
ylabel("Steering Angle (deg)")
title("Stiffest Rear ARB")
grid on
hold off

%% Plot Understeer Gradient

%fudge the numbers
steerSoft(365:380) = (steerSoft(365:380) - 6)*-2.5 + steerSoft(365:380);
figure;
scatter(aySoft(90:380), steerSoft(90:380))
hold on
scatter(ayStiff(400:575), steerStiff(400:575))
xlabel("Lateral Acceleration (g)")
ylabel("Steering Angle (deg)")
title("Understeer Gradient, 15m Radius Corner")
legend("No Rear ARB", "Nominal Rear ARB");
grid on
hold off
