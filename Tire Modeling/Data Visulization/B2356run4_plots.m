close all;   
clear;        
clc; 

addpath(genpath("../../Tire Modeling"))

dataFile = readtable("B2356run4_collapsed.csv");
SA = dataFile.SA*180/pi; 
FY = dataFile.FY*0.224809;
MZ = dataFile.MZ*0.73756;
%% Plot FY data


%IA = 0, P = 12psi, V = 25 mph
figure;
plot(SA(1:80),FY(1:80), '.','MarkerSize',15) %FZ = -250
hold on
plot(SA(81:160),FY(81:160), '.','MarkerSize',15) %FZ = -200
plot(SA(161:239),FY(161:239), '.','MarkerSize',15) %FZ = -150
plot(SA(401:480),FY(401:480), '.','MarkerSize',15) %FZ = -100
legend("FZ=-250lbf","FZ=-200lbf","FZ=-150lbf","FZ=-100lbf")
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
plot(SA(1041:1120),FY(1041:1120), '.','MarkerSize',15) %FZ = -50
legend("FZ=-250lbf","FZ = -200lbf","FZ=-150lbf","FZ=-50lbf")
title("IA = 4deg, P = 12psi, V = 25mph")
xlabel("SA (deg)")
ylabel("FY (lbf)")
grid on
hold off
%% FY model plots, data overlayed
P1=[250,1.4,2.4,-0.25,3,-0.1,-1.5,0,0,-30.5,1.15,1,0,0,-0.143,0,0,0,1.43];
L=[1,1,1,1,1,1,1,1];%0.62
alpha = linspace(-14,14,1000)*pi/180;
FZ = 150;
IA = 2*pi/180;


figure; %this plot compares model to raw data for various FZs, 2 IA
plot(alpha*180/pi,pacejka(P1,L,250,IA,alpha));
hold on
plot(alpha*180/pi,pacejka(P1,L,200,IA,alpha));
plot(alpha*180/pi,pacejka(P1,L,150,IA,alpha));
plot(alpha*180/pi,pacejka(P1,L,100,IA,alpha));
plot(alpha*180/pi,pacejka(P1,L,100,IA,alpha));
plot(alpha*180/pi,pacejka(P1,L,50,IA,alpha));
plot(SA(721:800),FY(721:800), '.','MarkerSize',15) %FZ = -250
plot(SA(481:560),FY(481:560), '.','MarkerSize',15) %FZ = -200
plot(SA(561:640),FY(561:640), '.','MarkerSize',15) %FZ = -150
plot(SA(801:880),FY(801:880), '.','MarkerSize',15) %FZ = -100
plot(SA(641:720),FY(641:720), '.','MarkerSize',15) %FZ = -50
legend("FZ=-250","FZ=-200","FZ=-150", "FZ=-100", "FZ=-50")
title("IA = 2deg, P = 12psi, V = 25mph")
xlabel("SA (deg)")
ylabel("FY (lbf)")
grid on
hold off

IA = 0*pi/180;
figure; %this plot also compares model to raw data for various FZs, 0 IA
plot(alpha*180/pi,pacejka(P1,L,250,IA,alpha));
hold on
plot(alpha*180/pi,pacejka(P1,L,200,IA,alpha));
plot(alpha*180/pi,pacejka(P1,L,150,IA,alpha));
plot(alpha*180/pi,pacejka(P1,L,100,IA,alpha));
plot(alpha*180/pi,pacejka(P1,L,50,IA,alpha));
plot(SA(1:80),FY(1:80), '.','MarkerSize',15) %FZ = -250
plot(SA(81:160),FY(81:160), '.','MarkerSize',15) %FZ = -200
plot(SA(161:239),FY(161:239), '.','MarkerSize',15) %FZ = -150
plot(SA(401:480),FY(401:480), '.','MarkerSize',15) %FZ = -100
legend("FZ=-250","FZ=-200","FZ=-150", "FZ=-100")
title("IA = 0deg, P = 12psi, V = 25mph")
xlabel("SA (deg)")
ylabel("FY (lbf)")
grid on
hold off


figure; %this plot compares model to raw data for various IAs
plot(alpha*180/pi,pacejka(P1,L,200,0,alpha));
hold on
plot(alpha*180/pi,pacejka(P1,L,200,2*pi/180,alpha));
plot(alpha*180/pi,pacejka(P1,L,200,4*pi/180,alpha));
plot(SA(81:160),FY(81:160), '.','MarkerSize',15) 
plot(SA(481:560),FY(481:560), '.','MarkerSize',15) 
plot(SA(881:960),FY(881:960), '.','MarkerSize',15) 
legend("IA=0","IA=2","IA=4")
title("FZ = -200lbf, P = 12psi, V = 25mph")
xlabel("SA (deg)")
ylabel("FY (lbf)")
grid on
hold off

%% FY vs FZ plots


figure; %this plot shows FY vs FZ at various SAs
FZ1 = linspace(0,300,1000);
plot(FZ1, pacejka(P1,L,FZ1,0,3.5*pi/180))
hold on
plot(FZ1, pacejka(P1,L,FZ1,0,3.6*pi/180))
plot(FZ1, pacejka(P1,L,FZ1,0,3.7*pi/180))
plot(FZ1, pacejka(P1,L,FZ1,0,4.4*pi/180))
plot(FZ1, pacejka(P1,L,FZ1,0,4.5*pi/180))
plot(FZ1, pacejka(P1,L,FZ1,0,4.6*pi/180))
legend("3.5","3.6","3.7","4.4","4.5","4.6")
xlabel("Normal Load (lbf)")
ylabel("FY (lbf)")
title("IA = 0deg, P = 12psi, V = 20mph")
grid on
hold off

%% Plot MZ data, model overlayed
PM=[250,2.5,0.16,0.12,0,-0.065,-0.8,0,0,4.8,1.8,0,0,0,0.2,-0.01,0,0.4,0,-0.045,250];
LM=[0.7,1,1,1,1,1,1,1];

%IA = 0, P = 12psi, V = 25 mph
figure;
plot(SA(1:80),MZ(1:80), '.','MarkerSize',15) %FZ = -250
hold on
plot(SA(81:160),MZ(81:160), '.','MarkerSize',15) %FZ = -200
plot(SA(161:239),MZ(161:239), '.','MarkerSize',15) %FZ = -150
plot(SA(401:480),MZ(401:480), '.','MarkerSize',15) %FZ = -100
plot(alpha*180/pi,pacejkaMZ(PM,L,250,0*pi/180,alpha));
plot(alpha*180/pi,pacejkaMZ(PM,L,200,0*pi/180,alpha));
plot(alpha*180/pi,pacejkaMZ(PM,L,150,0*pi/180,alpha));
plot(alpha*180/pi,pacejkaMZ(PM,L,100,0*pi/180,alpha));
legend("FZ=-250lbf","FZ=-200lbf","FZ=-150lbf","FZ=-100lbf")
title("IA = 0deg, P = 12psi, V = 25mph")
xlabel("SA (deg)")
ylabel("MZ (lbf*ft)")
grid on
hold off


%IA = 2, P = 12psi, V = 25 mph
figure;
plot(SA(721:800),MZ(721:800), '.','MarkerSize',15) %FZ = -250
hold on
plot(SA(481:560),MZ(481:560), '.','MarkerSize',15) %FZ = -200
plot(SA(561:640),MZ(561:640), '.','MarkerSize',15) %FZ = -150
plot(SA(801:880),MZ(801:880), '.','MarkerSize',15) %FZ = -100
plot(SA(641:720),MZ(641:720), '.','MarkerSize',15) %FZ = -50
plot(alpha*180/pi,pacejkaMZ(PM,L,250,2*pi/180,alpha));
plot(alpha*180/pi,pacejkaMZ(PM,L,200,2*pi/180,alpha));
plot(alpha*180/pi,pacejkaMZ(PM,L,150,2*pi/180,alpha));
plot(alpha*180/pi,pacejkaMZ(PM,L,100,2*pi/180,alpha));
plot(alpha*180/pi,pacejkaMZ(PM,L,50,2*pi/180,alpha));
legend("FZ=-250lbf","FZ=-200lbf","FZ=-150lbf","FZ=-100lbf","FZ=-50lbf")
title("IA = 2deg, P = 12psi, V = 25mph")
xlabel("SA (deg)")
ylabel("MZ (lbf*ft)")
grid on
hold off

%IA = 4, P = 12psi, V = 25 mph
figure;
plot(SA(1121:1200),MZ(1121:1200), '.','MarkerSize',15) %FZ = -250
hold on
plot(SA(881:960),MZ(881:960), '.','MarkerSize',15) %FZ = -200
plot(SA(961:1040),MZ(961:1040), '.','MarkerSize',15) %FZ = -150
plot(SA(1041:1120),MZ(1041:1120), '.','MarkerSize',15) %FZ = -50
plot(alpha*180/pi,pacejkaMZ(PM,L,250,4*pi/180,alpha));
plot(alpha*180/pi,pacejkaMZ(PM,L,200,4*pi/180,alpha));
plot(alpha*180/pi,pacejkaMZ(PM,L,150,4*pi/180,alpha));
plot(alpha*180/pi,pacejkaMZ(PM,L,100,4*pi/180,alpha));
plot(alpha*180/pi,pacejkaMZ(PM,L,50,4*pi/180,alpha));
legend("FZ=-250lbf","FZ = -200lbf","FZ=-150lbf","FZ=-50lbf")
title("IA = 4deg, P = 12psi, V = 25mph")
xlabel("SA (deg)")
ylabel("MZ (lbf*ft)")
grid on
hold off

%% Ackerman Steering Calcs
FZ_ackerman = [50,100,150,200,250,300];
FY_ackerman = zeros(6,1);
i_ackerman = zeros(6,1);
for i = 1:length(FZ_ackerman)
    [FY_ackerman(i), i_ackerman(i)] = max(pacejka(P1,L,FZ_ackerman(i),IA,alpha));
end
alpha_ackerman = alpha(i_ackerman)*-180/pi;

plot(FZ_ackerman, alpha_ackerman)

