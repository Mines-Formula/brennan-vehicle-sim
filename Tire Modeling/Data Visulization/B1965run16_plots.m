close all;   
clear;        
clc; 

addpath(genpath("../../Tire Modeling"))

dataFile = readtable("B1965run16 - Collapsed.csv");
SA_LCO = dataFile.SA*180/pi; 
FY_LCO = dataFile.FY*0.224809;
MZ_LCO = dataFile.MZ*0.73756;

dataFile = readtable("B2356run6 - Collapsed.csv");
SA = dataFile.SA*180/pi; 
FY = dataFile.FY*0.224809;
MZ = dataFile.MZ*0.73756;
%% Plot FY data
P_LC0 = [250, 1.17858, -2.45689, 0.207723, 0.00218554, 0.181269, 0.212474,...
    -2.41388, 0.605724, -30.9683, 1.44763, 0.0143188, -0.00587066,...
    -0.0126017, -0.00150596, -0.18861, -0.170156, 0.0185739, 0.0140275];

%P1=[250,3.0,1.76,-0.25,3,-2.23,-1.5,0,0,-30.5,1.15,1,0,0,-0.128,0,0,0,1.43];
L=[1,1,1,1,1,1,1,1];
alpha = linspace(-14,14,1000)*pi/180;
IA = 0*pi/180;

%IA = 0, P = 12psi, V = 25 mph
figure;
plot(SA_LCO(1:80),FY_LCO(1:80), '.','MarkerSize',15) %FZ = -250
hold on
plot(SA_LCO(81:160),FY_LCO(81:160), '.','MarkerSize',15) %FZ = -200
plot(SA_LCO(161:240),FY_LCO(161:240), '.','MarkerSize',15) %FZ = -150
plot(SA_LCO(401:480),FY_LCO(401:480), '.','MarkerSize',15) %FZ = -100
plot(SA_LCO(241:320),FY_LCO(241:320), '.','MarkerSize',15) %FZ = -50
plot(alpha*180/pi,pacejka(P_LC0,L,250,IA,alpha));
plot(alpha*180/pi,pacejka(P_LC0,L,200,IA,alpha));
plot(alpha*180/pi,pacejka(P_LC0,L,150,IA,alpha));
plot(alpha*180/pi,pacejka(P_LC0,L,100,IA,alpha));
plot(alpha*180/pi,pacejka(P_LC0,L,50,IA,alpha));
legend("FZ=-250lbf","FZ=-200lbf","FZ=-150lbf","FZ=-100lbf","FZ=-50lbf")
title("IA = 0deg, P = 12psi, V = 25mph")
xlabel("SA (deg)")
ylabel("FY (lbf)")
grid on
hold off

%IA = 2, P = 12psi, V = 25 mph
figure;
plot(SA_LCO(721:800),FY_LCO(721:800), '.','MarkerSize',15) %FZ = -250
hold on
plot(SA_LCO(481:560),FY_LCO(481:560), '.','MarkerSize',15) %FZ = -200
plot(SA_LCO(561:640),FY_LCO(561:640), '.','MarkerSize',15) %FZ = -150
plot(SA_LCO(801:880),FY_LCO(801:880), '.','MarkerSize',15) %FZ = -100
plot(SA_LCO(641:720),FY_LCO(641:720), '.','MarkerSize',15) %FZ = -50
legend("FZ=-250lbf","FZ=-200lbf","FZ=-150lbf","FZ=-100lbf","FZ=-50lbf")
title("IA = 2deg, P = 12psi, V = 25mph")
xlabel("SA (deg)")
ylabel("FY (lbf)")
grid on
hold off

%IA = 4, P = 12psi, V = 25 mph
figure;
plot(SA_LCO(1121:1200),FY_LCO(1121:1200), '.','MarkerSize',15) %FZ = -250
hold on
plot(SA_LCO(881:960),FY_LCO(881:960), '.','MarkerSize',15) %FZ = -200
plot(SA_LCO(961:1040),FY_LCO(961:1040), '.','MarkerSize',15) %FZ = -150
plot(SA_LCO(1201:1280),FY_LCO(1201:1280), '.','MarkerSize',15) %FZ = -100
plot(SA_LCO(1041:1120),FY_LCO(1041:1120), '.','MarkerSize',15) %FZ = -50
legend("FZ=-250lbf","FZ = -200lbf","FZ=-150lbf","FZ=-100lbf","FZ=-50lbf")
title("IA = 4deg, P = 12psi, V = 25mph")
xlabel("SA (deg)")
ylabel("FY (lbf)")
grid on
hold off

%IA = 0, P = 12psi, V = 45 mph
figure;
plot(SA_LCO(1921:2000),FY_LCO(1921:2000), '.','MarkerSize',15) %FZ = -250
hold on
plot(SA_LCO(1681:1760),FY_LCO(1681:1760), '.','MarkerSize',15) %FZ = -200
plot(SA_LCO(1761:1839),FY_LCO(1761:1839), '.','MarkerSize',15) %FZ = -150
plot(SA_LCO(2001:2080),FY_LCO(2001:2080), '.','MarkerSize',15) %FZ = -100
plot(SA_LCO(1841:1920),FY_LCO(1841:1920), '.','MarkerSize',15) %FZ = -50
legend("FZ=-250lbf","FZ=-200lbf","FZ=-150lbf","FZ=-100lbf","FZ=-50lbf")
title("IA = 0deg, P = 12psi, V = 45mph")
xlabel("SA (deg)")
ylabel("FY (lbf)")
grid on
hold off

%IA = 0, P = 12psi, V = 15 mph
figure;
plot(SA_LCO(1521:1600),FY_LCO(1521:1600), '.','MarkerSize',15) %FZ = -250
hold on
plot(SA_LCO(1281:1360),FY_LCO(1281:1360), '.','MarkerSize',15) %FZ = -200
plot(SA_LCO(1361:1439),FY_LCO(1761:1839), '.','MarkerSize',15) %FZ = -150
plot(SA_LCO(1601:1680),FY_LCO(1601:1680), '.','MarkerSize',15) %FZ = -100
plot(SA_LCO(1441:1520),FY_LCO(1441:1520), '.','MarkerSize',15) %FZ = -50
plot(alpha*180/pi,pacejka(P_LC0,L,250,IA,alpha));
plot(alpha*180/pi,pacejka(P_LC0,L,200,IA,alpha));
plot(alpha*180/pi,pacejka(P_LC0,L,150,IA,alpha));
plot(alpha*180/pi,pacejka(P_LC0,L,100,IA,alpha));
plot(alpha*180/pi,pacejka(P_LC0,L,50,IA,alpha));
legend("FZ=-250lbf","FZ=-200lbf","FZ=-150lbf","FZ=-100lbf","FZ=-50lbf")
title("IA = 0deg, P = 12psi, V =15mph")
xlabel("SA (deg)")
ylabel("FY (lbf)")
grid on
hold off
%% Compare LCO to R20
%IA = 0, P = 12psi, V = 25 mph
figure;
plot(SA_LCO(1:80),FY_LCO(1:80), '.','MarkerSize',15) %FZ = -250
hold on
plot(SA(1:80),FY(1:80), '*','MarkerSize',10) %FZ = -250
plot(SA_LCO(161:240),FY_LCO(161:240), '.','MarkerSize',15) %FZ = -150
plot(SA(161:240),FY(161:240), '*','MarkerSize',10) %FZ = -150
plot(SA_LCO(241:320),FY_LCO(241:320), '.','MarkerSize',15) %FZ = -50
plot(SA(241:320),FY(241:320), '*','MarkerSize',10) %FZ = -50
legend("LCO FZ=-250lbf","R20 FZ=-250lbf","LCO FZ=-150lbf","R20 FZ=-150lbf","LCO FZ=-50lbf","R20 FZ=-50lbf")
title("IA = 0deg, P = 12psi, V = 25mph")
xlabel("SA (deg)")
ylabel("FY (lbf)")
grid on
hold off

%IA = 0, P = 12psi, V = 45 mph
figure;
plot(SA_LCO(1921:2000),FY_LCO(1921:2000), '.','MarkerSize',15) %FZ = -250
hold on
plot(SA(1921:2000),FY(1921:2000), '*','MarkerSize',10) %FZ = -250
plot(SA_LCO(1761:1839),FY_LCO(1761:1839), '.','MarkerSize',15) %FZ = -150
plot(SA(1761:1839),FY(1761:1839), '*','MarkerSize',10) %FZ = -150
plot(SA_LCO(1841:1920),FY_LCO(1841:1920), '.','MarkerSize',15) %FZ = -50
plot(SA(1841:1920),FY(1841:1920), '*','MarkerSize',10) %FZ = -50
legend("LCO FZ=-250lbf","R20 FZ=-250lbf","LCO FZ=-150lbf","R20 FZ=-150lbf","LCO FZ=-50lbf","R20 FZ=-50lbf")
title("IA = 0deg, P = 12psi, V = 45mph")
xlabel("SA (deg)")
ylabel("FY (lbf)")
grid on
hold off

%IA = 0, P = 12psi, V = 15 mph
figure;
plot(SA_LCO(1521:1600),FY_LCO(1521:1600), '.','MarkerSize',15) %FZ = -250
hold on
plot(SA(1521:1600),FY(1521:1600), '*','MarkerSize',10) %FZ = -250
plot(SA_LCO(1361:1439),FY_LCO(1761:1839), '.','MarkerSize',15) %FZ = -150
plot(SA(1361:1439),FY(1761:1839), '*','MarkerSize',10) %FZ = -150
plot(SA_LCO(1441:1520),FY_LCO(1441:1520), '.','MarkerSize',15) %FZ = -50
plot(SA(1441:1520),FY(1441:1520), '*','MarkerSize',10) %FZ = -50
legend("LCO FZ=-250lbf","R20 FZ=-250lbf","LCO FZ=-150lbf","R20 FZ=-150lbf","LCO FZ=-50lbf","R20 FZ=-50lbf")
title("IA = 0deg, P = 12psi, V = 15mph")
xlabel("SA (deg)")
ylabel("FY (lbf)")
grid on
hold off
