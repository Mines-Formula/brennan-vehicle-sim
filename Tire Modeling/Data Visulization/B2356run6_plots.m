close all;   
clear;        
clc; 

addpath(genpath("../../Tire Modeling"))

dataFile = readtable("B2356run6 - Collapsed.csv");
SA = dataFile.SA*180/pi; 
FY = dataFile.FY*0.224809;
MZ = dataFile.MZ*0.73756;
%% Plot FY data
P1=[250,1.4,2.4,-0.25,3,-0.1,-1.5,0,0,-30.5,1.15,1,0,0,-0.128,0,0,0,1.43];
%P1=[250,3.0,1.76,-0.25,3,-2.23,-1.5,0,0,-30.5,1.15,1,0,0,-0.128,0,0,0,1.43];
L=[1,1,1,1,1,1,1,1];
alpha = linspace(-14,14,1000)*pi/180;
IA = 0*pi/180;

%IA = 0, P = 12psi, V = 25 mph
figure;
plot(SA(1:80),FY(1:80), '.','MarkerSize',15) %FZ = -250
hold on
plot(SA(81:160),FY(81:160), '.','MarkerSize',15) %FZ = -200
plot(SA(161:240),FY(161:240), '.','MarkerSize',15) %FZ = -150
plot(SA(401:480),FY(401:480), '.','MarkerSize',15) %FZ = -100
plot(SA(241:320),FY(241:320), '.','MarkerSize',15) %FZ = -50
plot(alpha*180/pi,pacejka(P1,L,250,IA,alpha));
plot(alpha*180/pi,pacejka(P1,L,200,IA,alpha));
plot(alpha*180/pi,pacejka(P1,L,150,IA,alpha));
plot(alpha*180/pi,pacejka(P1,L,100,IA,alpha));
plot(alpha*180/pi,pacejka(P1,L,50,IA,alpha));
legend("FZ=-250lbf","FZ=-200lbf","FZ=-150lbf","FZ=-100lbf","FZ=-50lbf")
title("IA = 0deg, P = 12psi, V = 25mph")
xlabel("SA (deg)")
ylabel("FY (lbf)")
grid on
hold off

%IA = 2, P = 12psi, V = 25 mph
figure;
plot(SA(721:800),FY(721:800), '.','MarkerSize',15) %FZ = -250
hold on
plot(SA(481:560),FY(481:560), '.','MarkerSize',15) %FZ = -200
plot(SA(561:640),FY(561:640), '.','MarkerSize',15) %FZ = -150
plot(SA(801:880),FY(801:880), '.','MarkerSize',15) %FZ = -100
plot(SA(641:720),FY(641:720), '.','MarkerSize',15) %FZ = -50
legend("FZ=-250lbf","FZ=-200lbf","FZ=-150lbf","FZ=-100lbf","FZ=-50lbf")
title("IA = 2deg, P = 12psi, V = 25mph")
xlabel("SA (deg)")
ylabel("FY (lbf)")
grid on
hold off

%IA = 4, P = 12psi, V = 25 mph
figure;
plot(SA(1121:1200),FY(1121:1200), '.','MarkerSize',15) %FZ = -250
hold on
plot(SA(881:960),FY(881:960), '.','MarkerSize',15) %FZ = -200
plot(SA(961:1040),FY(961:1040), '.','MarkerSize',15) %FZ = -150
plot(SA(1201:1280),FY(1201:1280), '.','MarkerSize',15) %FZ = -100
plot(SA(1041:1120),FY(1041:1120), '.','MarkerSize',15) %FZ = -50
legend("FZ=-250lbf","FZ = -200lbf","FZ=-150lbf","FZ=-100lbf","FZ=-50lbf")
title("IA = 4deg, P = 12psi, V = 25mph")
xlabel("SA (deg)")
ylabel("FY (lbf)")
grid on
hold off

%IA = 0, P = 12psi, V = 45 mph
figure;
plot(SA(1921:2000),FY(1921:2000), '.','MarkerSize',15) %FZ = -250
hold on
plot(SA(1681:1760),FY(1681:1760), '.','MarkerSize',15) %FZ = -200
plot(SA(1761:1839),FY(1761:1839), '.','MarkerSize',15) %FZ = -150
plot(SA(2001:2080),FY(2001:2080), '.','MarkerSize',15) %FZ = -100
plot(SA(1841:1920),FY(1841:1920), '.','MarkerSize',15) %FZ = -50
legend("FZ=-250lbf","FZ=-200lbf","FZ=-150lbf","FZ=-100lbf","FZ=-50lbf")
title("IA = 0deg, P = 12psi, V = 45mph")
xlabel("SA (deg)")
ylabel("FY (lbf)")
grid on
hold off

%IA = 0, P = 12psi, V = 15 mph
figure;
plot(SA(1521:1600),FY(1521:1600), '.','MarkerSize',15) %FZ = -250
hold on
plot(SA(1281:1360),FY(1281:1360), '.','MarkerSize',15) %FZ = -200
plot(SA(1361:1439),FY(1761:1839), '.','MarkerSize',15) %FZ = -150
plot(SA(1601:1680),FY(1601:1680), '.','MarkerSize',15) %FZ = -100
plot(SA(1441:1520),FY(1441:1520), '.','MarkerSize',15) %FZ = -50
legend("FZ=-250lbf","FZ=-200lbf","FZ=-150lbf","FZ=-100lbf","FZ=-50lbf")
title("IA = 0deg, P = 12psi, V =15mph")
xlabel("SA (deg)")
ylabel("FY (lbf)")
grid on
hold off