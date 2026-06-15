clear
clc
close all
addpath(genpath("Data Files"))
%this visualizaiton compares the effect of a damper change made on
%02072026. The front LSC was loosened by 1 rotation due to the front end
%losing it on bumps. 

%initial data (concentric circles test)
accelData1 = readtable("GX010168_HERO13 Black-ACCL.csv");
gpsData1 = readtable("GX010168_HERO13 Black-GPS9.csv");

%data after -1 turn front LSC damping
accelData2 = readtable("GX010173_HERO13 Black-ACCL.csv");
gpsData2 = readtable("GX010173_HERO13 Black-GPS9.csv");


 ax1 = accelData1.ax;
 ay1 = accelData1.ay;
 az1 = accelData1.az;

 ax2 = accelData2.ax;
 ay2 = accelData2.ay;
 az2 = accelData2.az;

%% Orient acceleration data
g = 9.81;
theta = acos(mean(az1,"omitnan")/g);
theta = 0.48;
ax_flat1 = ax1*cos(theta)-az1*sin(theta);
ax_flat2 = ax2*cos(theta)-az2*sin(theta);
%az_flat1 = az1*cos(theta)+ax1*sin(theta)-9.81;
%az_flat2 = az2*cos(theta)+ax2*sin(theta)-9.81;
az_flat1 = sqrt(az1.^2+ax1.^2)-9.81;
az_flat2 = sqrt(az2.^2+ax2.^2)-9.81;

accelData1.ax = ax_flat1;
accelData2.ax = ax_flat2;
accelData1.az = az_flat1;
accelData2.az = az_flat2;


%% Covert to timetables and merge data

%format time columns as time variables
accelData1.date = datetime(accelData1.date,...            
    'InputFormat','yyyy-MM-dd''T''HH:mm:ss.SSS''Z''',...
    'TimeZone','UTC');
gpsData1.date = datetime(gpsData1.date,...
    'InputFormat','yyyy-MM-dd''T''HH:mm:ss.SSS''Z''',...
    'TimeZone','UTC');

accelData2.date = datetime(accelData2.date,...            
    'InputFormat','yyyy-MM-dd''T''HH:mm:ss.SSS''Z''',...
    'TimeZone','UTC');
gpsData2.date = datetime(gpsData2.date,...
    'InputFormat','yyyy-MM-dd''T''HH:mm:ss.SSS''Z''',...
    'TimeZone','UTC');

%define time tables
accelTT1 = table2timetable(accelData1, "RowTimes","date");
accelTT1.date = accelTT1.date + seconds(0); %offset data to line shit up
gpsTT1 = table2timetable(gpsData1, "RowTimes","date");
mergedTT1 = synchronize(accelTT1,gpsTT1);  %merge time tables

accelTT2 = table2timetable(accelData2, "RowTimes","date");
accelTT2.date = accelTT2.date + seconds(0); %offset data to line shit up
gpsTT2 = table2timetable(gpsData2, "RowTimes","date");
mergedTT2 = synchronize(accelTT2,gpsTT2);  %merge time tables

%fill missing data
mergedTT1 = fillmissing(mergedTT1, 'linear');
mergedTT2 = fillmissing(mergedTT2, 'linear');

%%%%%% continue from here
%% Find timestamps
medium1 = timeRangeIndices(mergedTT1.date, 307.65, 333); %310-330
big1 = timeRangeIndices(mergedTT1.date, 330, 359); 

medium2 = timeRangeIndices(mergedTT2.date, 837, 865); 
big2 = timeRangeIndices(mergedTT2.date, 866, 887);



% Extract the relevant data based on the identified timestamps
ay_medium1 = mergedTT1.ay(medium1);
az_medium1 = mergedTT1.az(medium1);
ax_medium1 = mergedTT1.ax(medium1);
ay_big1 = mergedTT1.ay(big1);
az_big1 = mergedTT1.az(big1);

ay_medium2 = mergedTT2.ay(medium2);
az_medium2 = mergedTT2.az(medium2);
ay_big2 = mergedTT2.ay(big2);
az_big2 = mergedTT2.az(big2);


%% Smooth Data and define ellapsed time vectors
N = 250;  % number of samples for running average

ay_medium1 = movmean(ay_medium1, N); 
az_medium1 = movmean(az_medium1, N);
ax_medium1 = movmean(ax_medium1, N);
elapsedTime_medium1 = seconds(mergedTT1.date(medium1) - mergedTT1.date(medium1(1)));

ay_big1 = movmean(ay_big1, N); 
az_big1 = movmean(az_big1, N);
elapsedTime_big1 = seconds(mergedTT1.date(big1) - mergedTT1.date(big1(1)));

ay_medium2 = movmean(ay_medium2, N); 
az_medium2 = movmean(az_medium2, N);
elapsedTime_medium2 = seconds(mergedTT2.date(medium2) - mergedTT2.date(medium2(1)));

ay_big2 = movmean(ay_big2, N); 
az_big2 = movmean(az_big2, N);
elapsedTime_big2 = seconds(mergedTT2.date(big2) - mergedTT2.date(big2(1)));

%% Plot ay and az before and after damper change
figure;
plot(elapsedTime_medium1, ay_medium1)
hold on
plot(elapsedTime_medium1, az_medium1)
plot(elapsedTime_medium2, ay_medium2)
plot(elapsedTime_medium2, az_medium2)
xlabel('Time (s)');
ylabel('Acceleration (m/s^2)');
grid on
legend("a_y before", "a_z before", "a_y after", "a_z after");
title("40 ft Radius Skidpad")
hold off

figure;
tiledlayout(2,1) % 2 rows, 1 column

%% Top plot: a_z
nexttile
plot(elapsedTime_medium1, ay_medium1)
hold on
plot(elapsedTime_medium2, ay_medium2)
xlabel('Time (s)');
ylabel('a_y (m/s^2)');
grid on
legend("a_y before", "a_y after");
hold off


%% Bottom plot: a_y
nexttile
plot(elapsedTime_medium1, az_medium1)
hold on
plot(elapsedTime_medium2, az_medium2)
ylabel('a_z (m/s^2)');
grid on
legend("a_z before", "a_z after");
title("40 ft Radius Skidpad")
hold off