clear
clc
close all
addpath(genpath("Data Files"))
%this visualizaiton compares lateral acceleration for a 30ft, 40ft, and 50
%ft corner radius during tests conducted on 2-7-26

%austin data (concentric circles test)
accelData = readtable("GX010168_HERO13 Black-ACCL.csv");
gpsData = readtable("GX010168_HERO13 Black-GPS9.csv");

ay = accelData.ay;

%% Covert to timetables and merge data

%format time columns as time variables
accelData.date = datetime(accelData.date,...            
    'InputFormat','yyyy-MM-dd''T''HH:mm:ss.SSS''Z''',...
    'TimeZone','UTC');
gpsData.date = datetime(gpsData.date,...
    'InputFormat','yyyy-MM-dd''T''HH:mm:ss.SSS''Z''',...
    'TimeZone','UTC');

%define time tables
accelTT = table2timetable(accelData, "RowTimes","date");
accelTT.date = accelTT.date + seconds(0); %offset data to line shit up
gpsTT = table2timetable(gpsData, "RowTimes","date");
mergedTT = synchronize(accelTT,gpsTT);  %merge time tables

%fill missing data
mergedTT = fillmissing(mergedTT, 'linear');

%% Find timestamps
R30 = timeRangeIndices(mergedTT.date, 115, 121);
R40 = timeRangeIndices(mergedTT.date, 128, 134); 
R50 = timeRangeIndices(mergedTT.date, 142, 149); 

L30 = timeRangeIndices(mergedTT.date, 273, 278);
L40 = timeRangeIndices(mergedTT.date, 315, 322); 
L50 = timeRangeIndices(mergedTT.date, 340, 347); 



% Extract the relevant data based on the identified timestamps
ay_R30 = mergedTT.ay(R30);
ay_R40 = mergedTT.ay(R40);
ay_R50 = mergedTT.ay(R50);

ay_L30 = mergedTT.ay(L30);
ay_L40 = mergedTT.ay(L40);
ay_L50 = mergedTT.ay(L50);


%% Smooth Data and define ellapsed time vectors
N = 250;  % number of samples for running average

ay_R30 = movmean(ay_R30, N); 
elapsedTime_R30 = seconds(mergedTT.date(R30) - mergedTT.date(R30(1)));
ay_R40 = movmean(ay_R40, N); 
elapsedTime_R40 = seconds(mergedTT.date(R40) - mergedTT.date(R40(1)));
ay_R50 = movmean(ay_R50, N); 
elapsedTime_R50 = seconds(mergedTT.date(R50) - mergedTT.date(R50(1)));

ay_L30 = movmean(ay_L30, N); 
elapsedTime_L30 = seconds(mergedTT.date(L30) - mergedTT.date(L30(1)));
ay_L40 = movmean(ay_L40, N); 
elapsedTime_L40 = seconds(mergedTT.date(L40) - mergedTT.date(L40(1)));
ay_L50 = movmean(ay_L50, N); 
elapsedTime_L50 = seconds(mergedTT.date(L50) - mergedTT.date(L50(1)));
%% Plot ay for 30ft, 40ft, 50ft radius

figure; %turning right plot
plot(elapsedTime_R30, abs(ay_R30)/9.81)
hold on
plot(elapsedTime_R40, abs(ay_R40)/9.81)
plot(elapsedTime_R50, abs(ay_R50)/9.81)
xlabel('Time (s)');
ylabel(' Lateral Acceleration (g)');
grid on
legend("30ft Radius", "40ft Radius", "50ft Radius");
title("Lateral Acceleration at Various Skidpad Sizes (right)")
hold off

figure; %turning left plot
plot(elapsedTime_L30, abs(ay_L30)/9.81)
hold on
plot(elapsedTime_L40, abs(ay_L40)/9.81)
plot(elapsedTime_L50, abs(ay_L50)/9.81)
xlabel('Time (s)');
ylabel(' Lateral Acceleration (g)');
grid on
legend("30ft Radius", "40ft Radius", "50ft Radius");
title("Lateral Acceleration at Various Skidpad Sizes (left)")
hold off