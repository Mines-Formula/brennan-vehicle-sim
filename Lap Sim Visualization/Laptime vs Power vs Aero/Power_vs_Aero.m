clear
clc
close all

%each data set below sweeps power factor from 25% to 150%
noAero = readtable("base case.csv"); %CD=0.7, CL=0.15
%CLCD = 2.25 for all below
CL15 = readtable("CL15.csv"); %CL=1.5
CL2 = readtable("CL2.csv"); %CL=2
CL25 = readtable("CL25.csv"); %CL=2.5
CL3 = readtable("CL3.csv"); %CL=3
CL35 = readtable("CL35.csv"); %CL=3.5

noAero_times = noAero{1,2:end};
powerFactor = noAero{43,2:end}/100;

CL15_times = CL15{1,2:end};

CL2_times = CL2{1,2:end};

CL25_times = CL25{1,2:end};

CL3_times = CL3{1,2:end};

CL35_times = CL35{1,2:end};

%calculate power vector assuming baseline of 66 hp
power = 66*powerFactor; %power vector (hp)

figure
plot(power, noAero_times, 'k--', ...
     'LineWidth', 1.5);
hold on
plot(power, CL15_times, 'k-', ...
     'LineWidth', 1.5);
plot(power, CL25_times, 'k-', ...
     'LineWidth', 1.5);
plot(power, CL35_times, 'k-', ...
     'LineWidth', 1.5);

xlabel('Maximum Power Output (hp)');
ylabel('Simulated Endurance Laptime (s)');
title('Power vs Time for Different CL Values @ Constant CL/CD');
grid on;

% --- Add curve labels at right end ---
text(power(end), CL15_times(end), '  CL = 1.5');
text(power(end), CL25_times(end), '  CL = 2.5');
text(power(end), CL35_times(end),  '  CL = 3.5');
text(power(end), noAero_times(end),  '  No Aero Package');