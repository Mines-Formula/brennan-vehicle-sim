clear
clc
close all

%% Autocross & endurance (from OptimumLap)

%data sets below sweep mass from 500lb-660lb
endurance = readtable("endurance.csv");
autocross = readtable("autocross.csv");

endurance_times = endurance{1,2:end};
autocross_times = autocross{1,2:end};
endurance_mass = endurance{27,2:end};
autocross_mass = autocross{27,2:end};

endurance_times_norm = (endurance_times-endurance_times(11))/endurance_times(11);
autocross_times_norm = (autocross_times-autocross_times(11))/autocross_times(11);

%% Skidpad (using tire sensitivity assumption)
%mu from 
m = [500,520,540,560,580,600,620,640,660]/2.205;
mu = 1.5-0.00065*(m-272); %estimate mu using tire load sensitivity 
a = mu*9.81; %estimate max lateral acceleration (m/s^2)
R = 8.45; %skidpad radius (m)
D = 2*pi*R; %distance to travel (m)
V = sqrt(a*R); %calculate tangetial vehicle speed
skidpad_times = D./V;

skidpad_mass = m*2.205;
skidpad_times_norm = (skidpad_times-skidpad_times(6))/skidpad_times(6);


%% Accel (uses same tire sensitivity assumption)
accel_mass = skidpad_mass;
accel_times = [4.115,4.14,4.166,4.191,4.217,4.242,4.267,4.293,4.317];
accel_times_norm = (accel_times-accel_times(6))/accel_times(6);

%% Normalized Lap Time Sensitivity Plot

figure;
hold on;
grid on;
box on;

% Plot normalized times (convert to percent)
plot(endurance_mass, endurance_times_norm*100, 'k-', ...
    'LineWidth', 2);

plot(autocross_mass, autocross_times_norm*100, 'k--', ...
    'LineWidth', 2);

plot(skidpad_mass, skidpad_times_norm*100, 'k-.', ...
    'LineWidth', 2);

plot(accel_mass, accel_times_norm*100, 'k:', ...
    'LineWidth', 2);

% Zero reference line (nominal vehicle)
%yline(0,'k','LineWidth',0.5);

% Labels and title
xlabel('Vehicle Mass (lb)');
ylabel('Lap Time Change from Baseline (%)');
title('Dynamic Event Lap Time Sensitivity to Vehicle Mass');

% Legend
legend({'Endurance','Autocross','Skidpad','Acceleration'}, ...
    'Location','northwest');

% Axis formatting
set(gca,'FontSize',12);
xlim([500 660]);
ylim([-2.5,1.5])

hold off;

%% Normalized Point Sensitivity Plot

accel_scores = (accel_times-accel_times(6))*-64.7;
skidpad_scores = (skidpad_times-skidpad_times(6))*-71.86;
autocross_scores = (autocross_times-autocross_times(11))*-7.185;
endurance_scores = (endurance_times-endurance_times(11))*-5.39;

figure;
hold on;
grid on;
box on;

% Plot normalized times (convert to percent)
plot(endurance_mass, endurance_scores, 'k-', ...
    'LineWidth', 2);

plot(autocross_mass, autocross_scores, 'k--', ...
    'LineWidth', 2);

plot(skidpad_mass, skidpad_scores, 'k-.', ...
    'LineWidth', 2);

plot(accel_mass, accel_scores, 'k:', ...
    'LineWidth', 2);

% Zero reference line (nominal vehicle)
%yline(0,'k','LineWidth',0.5);

% Labels and title
xlabel('Vehicle Mass (lb)');
ylabel('Point Change from Baseline');
title('Dynamic Event Points Sensitivity to Vehicle Mass');

% Legend
legend({'Endurance','Autocross','Skidpad','Acceleration'}, ...
    'Location','northwest');

% Axis formatting
set(gca,'FontSize',12);
xlim([500 660]);
ylim([-2.5,1.5])

hold off;