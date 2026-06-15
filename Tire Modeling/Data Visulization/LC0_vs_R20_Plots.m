close all;   
clear;        
clc; 

addpath(genpath("../../Tire Modeling"))

%R20:
S_R20_4 = load('Data Files/A2356run4'); %load data as struct
S_R20_4 = rmfield(S_R20_4, {'channel', 'source', 'testid', 'tireid'}); %remove these fields
T_R20_4 = struct2table(S_R20_4); %covert struct 2 table

S_R20_5 = load('Data Files/A2356run5'); %load data as struct
S_R20_5 = rmfield(S_R20_5, {'channel', 'source', 'testid', 'tireid'}); %remove these fields
T_R20_5 = struct2table(S_R20_5); %covert struct 2 table

S_R20_6 = load('Data Files/A2356run6'); %load data as struct
S_R20_6 = rmfield(S_R20_6, {'channel', 'source', 'testid', 'tireid'}); %remove these fields
T_R20_6 = struct2table(S_R20_6); %covert struct 2 table

%LC0:
S_LC0_18 = load('Data Files/A1965run18'); %load data as struct
S_LC0_18 = rmfield(S_LC0_18, {'channel', 'source', 'testid', 'tireid'}); %remove these fields
T_LC0_18 = struct2table(S_LC0_18); %covert struct 2 table

S_LC0_19 = load('Data Files/A1965run19'); %load data as struct
S_LC0_19 = rmfield(S_LC0_19, {'channel', 'source', 'testid', 'tireid'}); %remove these fields
T_LC0_19 = struct2table(S_LC0_19); %covert struct 2 table

N = 5; %running average factor
T_R20_4_sm = varfun(@(x) movmean(x,N), T_R20_4); %smooth data with running average
T_R20_4_sm.Properties.VariableNames = T_R20_4.Properties.VariableNames; %set variable names T_sm
T_R20_5_sm = varfun(@(x) movmean(x,N), T_R20_5); %smooth data with running average
T_R20_5_sm.Properties.VariableNames = T_R20_5.Properties.VariableNames; %set variable names T_sm
T_LC0_18_sm = varfun(@(x) movmean(x,N), T_LC0_18); %smooth data with running average
T_LC0_18_sm.Properties.VariableNames = T_LC0_18.Properties.VariableNames; %set variable names T_sm
T_R20_6_sm = varfun(@(x) movmean(x,N), T_R20_6); %smooth data with running average
T_R20_6_sm.Properties.VariableNames = T_R20_6.Properties.VariableNames; %set variable names T_sm
T_LC0_19_sm = varfun(@(x) movmean(x,N), T_LC0_19); %smooth data with running average
T_LC0_19_sm.Properties.VariableNames = T_LC0_19.Properties.VariableNames; %set variable names T_sm


iR20_4 = linspace(1,length(T_R20_4_sm.ET),length(T_R20_4_sm.ET)); %index vector
iR20_5 = linspace(1,length(T_R20_5_sm.ET),length(T_R20_5_sm.ET)); %index vector
iR20_6 = linspace(1,length(T_R20_6_sm.ET),length(T_R20_6_sm.ET)); %index vector
iLC0_18 = linspace(1,length(T_LC0_18_sm.ET),length(T_LC0_18_sm.ET)); %index vector
iLC0_19 = linspace(1,length(T_LC0_19_sm.ET),length(T_LC0_19_sm.ET)); %index vector

%% Compare initial and final SA sweeps for LC0 vs R20

figure;
scatter(T_R20_4_sm.SA(6255:7418),T_R20_4_sm.FY(6255:7418), "b", '.')
hold on
title("Initial vs Final Run For LC0 and R20 (250lbf FZ, 0 IA, 12 psi)")
scatter(T_R20_6_sm.SA(24933:26158),T_R20_6_sm.FY(24933:26158), "g", '.')
scatter(T_LC0_18_sm.SA(5014:6233),T_LC0_18_sm.FY(5014:6233), "r", '.')
scatter(T_LC0_19_sm.SA(25041:26233),T_LC0_19_sm.FY(25041:26233), "m", '.')
ylabel("FY (lbf)")
xlabel("SA (deg)")
legend("R20 Initial", "R20 Final", "LC0 Initial", "LC0 Final")
grid on
ylim([-650,650])
hold off

%% LC0 vs R20 at each load, 0IA 12PSI (initial run)
figure;
scatter(T_R20_4_sm.SA(6255:7418),T_R20_4_sm.FY(6255:7418), "b", '.') %250
%scatter(T_R20_4_sm.SA(1:2484),T_R20_4_sm.FY(1:2484), "b", '.') %250
hold on
title("Initial Run for LC0 (red) and R20 (blue) (0 IA, 12 psi)")
scatter(T_R20_4_sm.SA(2546:3667),T_R20_4_sm.FY(2546:3667), "b", '.') %200
scatter(T_R20_4_sm.SA(3754:4937),T_R20_4_sm.FY(3754:4937), "b", '.') %150
scatter(T_R20_4_sm.SA(7491:8645),T_R20_4_sm.FY(7491:8645), "b", '.') %100
scatter(T_R20_4_sm.SA(5006:6220),T_R20_4_sm.FY(5006:6220), "b", '.') %50
scatter(T_LC0_18_sm.SA(5014:6233),T_LC0_18_sm.FY(5014:6233), "r", '.') %250
%scatter(T_LC0_18_sm.SA(1:1235),T_LC0_18_sm.FY(1:1235), "r", '.') %250
scatter(T_LC0_18_sm.SA(1300:2480),T_LC0_18_sm.FY(1300:2480), "r", '.') %200
scatter(T_LC0_18_sm.SA(2568:3737),T_LC0_18_sm.FY(2568:3737), "r", '.') %150
scatter(T_LC0_18_sm.SA(6330:7477),T_LC0_18_sm.FY(6330:7477), "r", '.') %100
scatter(T_LC0_18_sm.SA(3767:4985),T_LC0_18_sm.FY(3767:4985), "r", '.') %100
ylabel("FY (lbf)")
xlabel("SA (deg)")
grid on
ylim([-650,650])
hold off

%% LC0 vs R20 at each load, 0IA 12PSI (final run)
figure;
scatter(T_R20_6_sm.SA(24933:26158),T_R20_6_sm.FY(24933:26158), "b", '.') %250
%scatter(T_R20_6_sm.SA(19947:21171),T_R20_6_sm.FY(19947:21171), "b", '.') %250
hold on
title("Final Run for LC0 (red) and R20 (blue) (0 IA, 12 psi)")
scatter(T_R20_6_sm.SA(21195:22363),T_R20_6_sm.FY(21195:22363), "b", '.') %200
scatter(T_R20_6_sm.SA(22439:23646),T_R20_6_sm.FY(22439:23646), "b", '.') %150
scatter(T_R20_6_sm.SA(26192:27334),T_R20_6_sm.FY(26192:27334), "b", '.') %100
scatter(T_R20_6_sm.SA(23698:24905),T_R20_6_sm.FY(23698:24905), "b", '.') %50
scatter(T_LC0_19_sm.SA(25041:26233),T_LC0_19_sm.FY(25041:26233), "r", '.') %250
%scatter(T_LC0_19_sm.SA(20020:21235),T_LC0_19_sm.FY(20020:21235), "r", '.') %250
scatter(T_LC0_19_sm.SA(21311:22471),T_LC0_19_sm.FY(21311:22471), "r", '.') %200
scatter(T_LC0_19_sm.SA(22508:23737),T_LC0_19_sm.FY(22508:23737), "r", '.') %150
scatter(T_LC0_19_sm.SA(26256:27487),T_LC0_19_sm.FY(26256:27487), "r", '.') %100
scatter(T_LC0_19_sm.SA(23760:24983),T_LC0_19_sm.FY(23760:24983), "r", '.') %100
ylabel("FY (lbf)")
xlabel("SA (deg)")
grid on
ylim([-650,650])
hold off

%% LC0 vs R20 at each load, 0IA 10PSI
figure;
%scatter(T_R20_4_sm.SA(21192:22414),T_R20_4_sm.FY(21192:22414), "b", '.') %250
scatter(T_R20_4_sm.SA(26177:27399),T_R20_4_sm.FY(26177:27399), "b", '.') %250
hold on
title("LC0 (red) and R20 (blue) (0 IA, 10 psi)")
scatter(T_R20_4_sm.SA(22438:23655),T_R20_4_sm.FY(22438:23655), "b", '.') %200
scatter(T_R20_4_sm.SA(23681:24900),T_R20_4_sm.FY(23681:24900), "b", '.') %150
scatter(T_R20_4_sm.SA(27433:28640),T_R20_4_sm.FY(27433:28640), "b", '.') %100
scatter(T_R20_4_sm.SA(24927:26153),T_R20_4_sm.FY(24927:26153), "b", '.') %50
%scatter(T_LC0_18_sm.SA(25040:26209),T_LC0_18_sm.FY(25040:26209), "r", '.') %250
scatter(T_LC0_18_sm.SA(20046:21213),T_LC0_18_sm.FY(20046:21213), "r", '.') %250
scatter(T_LC0_18_sm.SA(21280:22461),T_LC0_18_sm.FY(21280:22461), "r", '.') %200
scatter(T_LC0_18_sm.SA(22552:23693),T_LC0_18_sm.FY(22552:23693), "r", '.') %150
scatter(T_LC0_18_sm.SA(26247:27468),T_LC0_18_sm.FY(26247:27468), "r", '.') %100
scatter(T_LC0_18_sm.SA(23749:24597),T_LC0_18_sm.FY(23749:24597), "r", '.') %100
ylabel("FY (lbf)")
xlabel("SA (deg)")
grid on
ylim([-650,650])
hold off

%% LC0 vs R20 at each load, 0IA 14PSI
figure;
%scatter(T_R20_5_sm.SA(16206:17433),T_R20_5_sm.FY(16206:17433), "b", '.') %250
scatter(T_R20_5_sm.SA(11230:12499),T_R20_5_sm.FY(11230:12499), "b", '.') %250
hold on
title("LC0 (red) and R20 (blue) (0 IA, 14 psi)")
scatter(T_R20_5_sm.SA(12470:13695),T_R20_5_sm.FY(12470:13695), "b", '.') %200
scatter(T_R20_5_sm.SA(13716:14941),T_R20_5_sm.FY(13716:14941), "b", '.') %150
scatter(T_R20_5_sm.SA(17453:18672),T_R20_5_sm.FY(17453:18672), "b", '.') %100
scatter(T_R20_5_sm.SA(14961:16186),T_R20_5_sm.FY(14961:16186), "b", '.') %50
%scatter(T_LC0_18_sm.SA(45019:46209),T_LC0_18_sm.FY(45019:46209), "r", '.') %250
scatter(T_LC0_18_sm.SA(40009:41207),T_LC0_18_sm.FY(40009:41207), "r", '.') %250
scatter(T_LC0_18_sm.SA(41285:42455),T_LC0_18_sm.FY(41285:42455), "r", '.') %200
scatter(T_LC0_18_sm.SA(42482:43705),T_LC0_18_sm.FY(42482:43705), "r", '.') %150
scatter(T_LC0_18_sm.SA(46239:47419),T_LC0_18_sm.FY(46239:47419), "r", '.') %100
scatter(T_LC0_18_sm.SA(43742:44960),T_LC0_18_sm.FY(43742:44960), "r", '.') %100
ylabel("FY (lbf)")
xlabel("SA (deg)")
grid on
ylim([-650,650])
hold off

%% LC0 vs R20 at each load, 0IA 8PSI
figure;
%scatter(T_R20_6_sm.SA(4997:6222),T_R20_6_sm.FY(4997:6222), "b", '.') %250
scatter(T_R20_6_sm.SA(1:1236),T_R20_6_sm.FY(1:1236), "b", '.') %250
hold on
title("LC0 (red) and R20 (blue) (0 IA, 8 psi)")
scatter(T_R20_6_sm.SA(1259:2420),T_R20_6_sm.FY(1259:2420), "b", '.') %200
scatter(T_R20_6_sm.SA(2506:3704),T_R20_6_sm.FY(2506:3704), "b", '.') %150
scatter(T_R20_6_sm.SA(6248:7451),T_R20_6_sm.FY(6248:7451), "b", '.') %100
scatter(T_R20_6_sm.SA(3769:4977),T_R20_6_sm.FY(3769:4977), "b", '.') %50
%scatter(T_LC0_19_sm.SA(5034:6239),T_LC0_19_sm.FY(5034:6239), "r", '.') %250
scatter(T_LC0_19_sm.SA(1:1224),T_LC0_19_sm.FY(1:1224), "r", '.') %250
scatter(T_LC0_19_sm.SA(1296:2465),T_LC0_19_sm.FY(1296:2465), "r", '.') %200
scatter(T_LC0_19_sm.SA(2518:3679),T_LC0_19_sm.FY(2518:3679), "r", '.') %150
scatter(T_LC0_19_sm.SA(6266:7468),T_LC0_19_sm.FY(6266:7468), "r", '.') %100
scatter(T_LC0_19_sm.SA(3764:4980),T_LC0_19_sm.FY(3764:4980), "r", '.') %100
ylabel("FY (lbf)")
xlabel("SA (deg)")
grid on
ylim([-650,650])
hold off

%% R20 at each pressure, 0IA FZ = 250
figure;
scatter(T_R20_6_sm.SA(4997:6222),T_R20_6_sm.FY(4997:6222), "b", '.') %8
hold on
scatter(T_R20_4_sm.SA(21192:22414),T_R20_4_sm.FY(21192:22414), "g", '.') %10
title("R20 Response to Tire Pressure (0 IA, FZ = 250lbf")
scatter(T_R20_4_sm.SA(6255:7418),T_R20_4_sm.FY(6255:7418), "r", '.') %12
scatter(T_R20_5_sm.SA(11230:12449),T_R20_5_sm.FY(11230:12449), "m", '.') %14
ylabel("FY (lbf)")
xlabel("SA (deg)")
legend("8 psi", "10 psi", "12 psi", "14 psi")
grid on
ylim([-650,650])
hold off

%% LC0 at each pressure, 0IA FZ = 250
figure;
scatter(T_LC0_19_sm.SA(5034:6239),T_LC0_19_sm.FY(5034:6239), "b", '.') %8
hold on
scatter(T_LC0_18_sm.SA(25040:26209),T_LC0_18_sm.FY(25040:26209), "g", '.') %10
title("LC0 Response to Tire Pressure (0 IA, FZ = 250lbf")
scatter(T_LC0_18_sm.SA(5014:6233),T_LC0_18_sm.FY(5014:6233), "r", '.') %12
scatter(T_LC0_18_sm.SA(45019:46209),T_LC0_18_sm.FY(45019:46209), "m", '.') %14
ylabel("FY (lbf)")
xlabel("SA (deg)")
legend("8 psi", "10 psi", "12 psi", "14 psi")
grid on
ylim([-650,650])
hold off
%% LC0 Model vs Data
P_LC0 = [250, 1.17858, -2.45689, 0.207723, 0.00218554, 0.181269, 0.212474,...
    -2.41388, 0.605724, -30.9683, 1.44763, 0.0143188, -0.00587066,...
    -0.0126017, -0.00150596, -0.18861, -0.170156, 0.0185739, 0.0140275];
L=[1,1,1,1,1,1,1,1]; %scaling factors
alpha = linspace(-14,14,1000)*pi/180;
IA = 0*pi/180;

%0IA, 12PSI (final run)
figure;
scatter(T_LC0_19_sm.SA(20020:21235),T_LC0_19_sm.FY(20020:21235), "b", '.') %250
hold on
scatter(T_LC0_19_sm.SA(21311:22471),T_LC0_19_sm.FY(21311:22471), "r", '.') %200
scatter(T_LC0_19_sm.SA(22508:23737),T_LC0_19_sm.FY(22508:23737), "g", '.') %150
scatter(T_LC0_19_sm.SA(26256:27487),T_LC0_19_sm.FY(26256:27487), "y", '.') %100
scatter(T_LC0_19_sm.SA(23760:24983),T_LC0_19_sm.FY(23760:24983), "m", '.') %50
plot(alpha*180/pi,pacejka(P_LC0,L,250,IA,alpha));
plot(alpha*180/pi,pacejka(P_LC0,L,200,IA,alpha));
plot(alpha*180/pi,pacejka(P_LC0,L,150,IA,alpha));
plot(alpha*180/pi,pacejka(P_LC0,L,100,IA,alpha));
plot(alpha*180/pi,pacejka(P_LC0,L,50,IA,alpha));
title("LC0 Model vs Data, Final Run (0 IA, 12 psi)")
legend("FZ = 250lbf", "FZ = 200lbf", "FZ = 150lbf", "FZ = 100lbf", "FZ = 50lbf")
ylabel("FY (lbf)")
xlabel("SA (deg)")
grid on
hold off

%0, 2, 4IA, 12PSI (final run)
figure;
scatter(T_LC0_19_sm.SA(20020:21235),T_LC0_19_sm.FY(20020:21235), "b", '.') %250, 0IA
hold on
scatter(T_LC0_19_sm.SA(31260:32574),T_LC0_19_sm.FY(31260:32574), "g", '.') %250, 2IA
scatter(T_LC0_19_sm.SA(37539:38732),T_LC0_19_sm.FY(37539:38732), "r", '.') %250, 4IA


scatter(T_LC0_19_sm.SA(22508:23737),T_LC0_19_sm.FY(22508:23737), "b", '.') %150, 0IA
scatter(T_LC0_19_sm.SA(28757:29959),T_LC0_19_sm.FY(28757:29959), "g", '.') %150, 2IA
scatter(T_LC0_19_sm.SA(35068:36174),T_LC0_19_sm.FY(35068:36174), "r", '.') %150, 4IA

scatter(T_LC0_19_sm.SA(23760:24983),T_LC0_19_sm.FY(23760:24983), "b", '.') %50, 0IA
scatter(T_LC0_19_sm.SA(30007:31235),T_LC0_19_sm.FY(30007:31235), "g", '.') %50, 0IA
scatter(T_LC0_19_sm.SA(36270:37481),T_LC0_19_sm.FY(36270:37481), "r", '.') %50, 0IA
plot(alpha*180/pi,pacejka(P_LC0,L,250,0,alpha));
plot(alpha*180/pi,pacejka(P_LC0,L,250,2,alpha));
plot(alpha*180/pi,pacejka(P_LC0,L,250,4,alpha));
plot(alpha*180/pi,pacejka(P_LC0,L,150,0,alpha));
plot(alpha*180/pi,pacejka(P_LC0,L,150,2,alpha));
plot(alpha*180/pi,pacejka(P_LC0,L,150,4,alpha));
plot(alpha*180/pi,pacejka(P_LC0,L,50,0,alpha));
plot(alpha*180/pi,pacejka(P_LC0,L,50,2,alpha));
plot(alpha*180/pi,pacejka(P_LC0,L,50,4,alpha));
title("LC0 Model vs Data, Final Run (12 psi)")
legend("0IA", "2IA", "4IA")
ylabel("FY (lbf)")
xlabel("SA (deg)")
grid on
hold off

%0IA, 12PSI (initial run)
figure;
scatter(T_LC0_18_sm.SA(5014:6233), T_LC0_18_sm.FY(5014:6233), "b", '.') %250
hold on
scatter(T_LC0_18_sm.SA(1300:2480), T_LC0_18_sm.FY(1300:2480), "r", '.') %200
scatter(T_LC0_18_sm.SA(2568:3737), T_LC0_18_sm.FY(2568:3737), "g", '.') %150
scatter(T_LC0_18_sm.SA(6330:7477), T_LC0_18_sm.FY(6330:7477), "y", '.') %100
scatter(T_LC0_18_sm.SA(3767:4985), T_LC0_18_sm.FY(3767:4985), "m", '.') %50
plot(alpha*180/pi, pacejka(P_LC0, L, 250, IA, alpha));
plot(alpha*180/pi, pacejka(P_LC0, L, 200, IA, alpha));
plot(alpha*180/pi, pacejka(P_LC0, L, 150, IA, alpha));
plot(alpha*180/pi, pacejka(P_LC0, L, 100, IA, alpha));
plot(alpha*180/pi, pacejka(P_LC0, L, 50, IA, alpha));
title("LC0 Model vs Data, Initial Run (0 IA, 12 psi)")
legend("FZ = 250lbf", "FZ = 200lbf", "FZ = 150lbf", "FZ = 100lbf", "FZ = 50")
ylabel("FY (lbf)")
xlabel("SA (deg)")
grid on
hold off

%0, 2, 4IA, 12PSI (initial run)
figure;
%250 lbf
scatter(T_LC0_18_sm.SA(20046:21213), T_LC0_18_sm.FY(20046:21213), "b", '.') %250, 0IA
hold on
scatter(T_LC0_18_sm.SA(31291:32485), T_LC0_18_sm.FY(31291:32485), "g", '.') %250, 2IA
scatter(T_LC0_18_sm.SA(37519:38659), T_LC0_18_sm.FY(37519:38659), "r", '.') %250, 4IA
%150 lbf
scatter(T_LC0_18_sm.SA(22552:23693), T_LC0_18_sm.FY(22552:23693), "b", '.') %150, 0IA
scatter(T_LC0_18_sm.SA(28787:29962), T_LC0_18_sm.FY(28787:29962), "g", '.') %150, 2IA
scatter(T_LC0_18_sm.SA(35025:36211), T_LC0_18_sm.FY(35025:36211), "r", '.') %150, 4IA
%50 lbf
scatter(T_LC0_18_sm.SA(23749:24957), T_LC0_18_sm.FY(23749:24957), "b", '.') %50, 0IA
scatter(T_LC0_18_sm.SA(29997:31214), T_LC0_18_sm.FY(29997:31214), "g", '.') %50, 2IA
scatter(T_LC0_18_sm.SA(36301:37465), T_LC0_18_sm.FY(36301:37465), "r", '.') %50, 4IA
% Pacejka overlays
plot(alpha*180/pi, pacejka(P_LC0, L, 250, 0, alpha));
plot(alpha*180/pi, pacejka(P_LC0, L, 250, 2, alpha));
plot(alpha*180/pi, pacejka(P_LC0, L, 250, 4, alpha));
plot(alpha*180/pi, pacejka(P_LC0, L, 150, 0, alpha));
plot(alpha*180/pi, pacejka(P_LC0, L, 150, 2, alpha));
plot(alpha*180/pi, pacejka(P_LC0, L, 150, 4, alpha));
plot(alpha*180/pi, pacejka(P_LC0, L, 50, 0, alpha));
plot(alpha*180/pi, pacejka(P_LC0, L, 50, 2, alpha));
plot(alpha*180/pi, pacejka(P_LC0, L, 50, 4, alpha));
title("LC0 Model vs Data, Initial Run (12 psi)")
legend("0° IA", "2° IA", "4° IA")
ylabel("FY (lbf)")
xlabel("SA (deg)")
grid on
hold off

%% R20 Model vs Data
P_R20 = [250, 1.59646, 2.36421, -0.174142, 0.00296546, 0.291861,...
    -0.485933, -0.236369, 0.194122, -30.8375, 1.3082, 0.00798175,...
    -0.00123245, -0.00103767, -0.0011476, -0.0283671, 0, 0.0292794, -0.00139734];
%0 IA, 12 psi (R20 Run 4)
figure;
scatter(T_R20_4_sm.SA(2484:7418), T_R20_4_sm.FY(2484:7418), "b", '.') %250
hold on

scatter(T_R20_4_sm.SA(2546:3667), T_R20_4_sm.FY(2546:3667), "r", '.')  %200
scatter(T_R20_4_sm.SA(3754:4937), T_R20_4_sm.FY(3754:4937), "g", '.')  %150
scatter(T_R20_4_sm.SA(7491:8645), T_R20_4_sm.FY(7491:8645), "y", '.')  %100
scatter(T_R20_4_sm.SA(5006:6220), T_R20_4_sm.FY(5006:6220), "m", '.')  %50

plot(alpha*180/pi, pacejka(P_R20, L, 250, 0, alpha));
plot(alpha*180/pi, pacejka(P_R20, L, 200, 0, alpha));
plot(alpha*180/pi, pacejka(P_R20, L, 150, 0, alpha));
plot(alpha*180/pi, pacejka(P_R20, L, 100, 0, alpha));
plot(alpha*180/pi, pacejka(P_R20, L, 50, 0, alpha));

title("R20 Model vs Data Initial (0° IA, 12 psi)")
legend("250", "200", "150", "100", "50")
ylabel("FY (lbf)")
xlabel("SA (deg)")
grid on
hold off

%0, 2, 4 IA at 12 psi (R20 Run 4)
figure;

% 250 lbf
scatter(T_R20_4_sm.SA(6255:7418),     T_R20_4_sm.FY(6255:7418), "b", '.') %0 IA
hold on
scatter(T_R20_4_sm.SA(12470:13695),   T_R20_4_sm.FY(12470:13695), "g", '.') %2 IA
scatter(T_R20_4_sm.SA(18696:19922),   T_R20_4_sm.FY(18696:19922), "r", '.') %4 IA

% 150 lbf
scatter(T_R20_4_sm.SA(3754:4937),     T_R20_4_sm.FY(3754:4937), "b", '.') %0 IA
scatter(T_R20_4_sm.SA(10000:11183),    T_R20_4_sm.FY(10000:11183), "g", '.') %2 IA
scatter(T_R20_4_sm.SA(16205:17419),   T_R20_4_sm.FY(16205:17419), "r", '.') %4 IA

% 50 lbf
scatter(T_R20_4_sm.SA(5006:6220),     T_R20_4_sm.FY(5006:6220), "b", '.') %0 IA
scatter(T_R20_4_sm.SA(11224:12388),   T_R20_4_sm.FY(11224:12388), "g", '.') %2 IA
scatter(T_R20_4_sm.SA(17450:18674),   T_R20_4_sm.FY(17450:18674), "r", '.') %4 IA

% Model curves
plot(alpha*180/pi, pacejka(P_R20, L, 250, 0, alpha));
plot(alpha*180/pi, pacejka(P_R20, L, 250, 2, alpha));
plot(alpha*180/pi, pacejka(P_R20, L, 250, 4, alpha));

plot(alpha*180/pi, pacejka(P_R20, L, 150, 0, alpha));
plot(alpha*180/pi, pacejka(P_R20, L, 150, 2, alpha));
plot(alpha*180/pi, pacejka(P_R20, L, 150, 4, alpha));

plot(alpha*180/pi, pacejka(P_R20, L, 50, 0, alpha));
plot(alpha*180/pi, pacejka(P_R20, L, 50, 2, alpha));
plot(alpha*180/pi, pacejka(P_R20, L, 50, 4, alpha));

title("R20 Model vs Data  Initial (12 psi, IA Sweep)")
legend("250 0°", "250 2°", "250 4°")
ylabel("FY (lbf)")
xlabel("SA (deg)")
grid on
hold off

% 0 IA, 12 psi (R20 Run 6)
figure;
scatter(T_R20_6_sm.SA(24933:26158), T_R20_6_sm.FY(24933:26158), "b", '.') %250
hold on

scatter(T_R20_6_sm.SA(21195:22363), T_R20_6_sm.FY(21195:22363), "r", '.') %200
scatter(T_R20_6_sm.SA(22439:23646), T_R20_6_sm.FY(22439:23646), "g", '.') %150
scatter(T_R20_6_sm.SA(26192:27334), T_R20_6_sm.FY(26192:27334), "y", '.') %100
scatter(T_R20_6_sm.SA(23698:24905), T_R20_6_sm.FY(23698:24905), "m", '.') %50

plot(alpha*180/pi, pacejka(P_R20, L, 250, 0, alpha));
plot(alpha*180/pi, pacejka(P_R20, L, 200, 0, alpha));
plot(alpha*180/pi, pacejka(P_R20, L, 150, 0, alpha));
plot(alpha*180/pi, pacejka(P_R20, L, 100, 0, alpha));
plot(alpha*180/pi, pacejka(P_R20, L, 50, 0, alpha));

title("R20 Run 6 vs Model (0° IA, 12 psi)")
legend("250", "200", "150", "100", "50")
ylabel("FY (lbf)")
xlabel("SA (deg)")
grid on
hold off

% 0, 2, 4 IA at 12 psi (R20 Run 6)
figure;

%% 250 lbf
scatter(T_R20_6_sm.SA(24933:26158), T_R20_6_sm.FY(24933:26158), "b", '.') %0°
hold on
scatter(T_R20_6_sm.SA(31163:32222), T_R20_6_sm.FY(31163:32222), "g", '.') %2°
scatter(T_R20_6_sm.SA(37387:38458), T_R20_6_sm.FY(37387:38458), "r", '.') %4°

%% 150 lbf
scatter(T_R20_6_sm.SA(22439:23646), T_R20_6_sm.FY(22439:23646), "b", '.') %0°
scatter(T_R20_6_sm.SA(28669:29848), T_R20_6_sm.FY(28669:29848), "g", '.') %2°
scatter(T_R20_6_sm.SA(34900:36121), T_R20_6_sm.FY(34900:36121), "r", '.') %4°

%% 50 lbf
scatter(T_R20_6_sm.SA(23698:24905), T_R20_6_sm.FY(23698:24905), "b", '.') %0°
scatter(T_R20_6_sm.SA(29941:31111), T_R20_6_sm.FY(29941:31111), "g", '.') %2°
scatter(T_R20_6_sm.SA(36141:37338), T_R20_6_sm.FY(36141:37338), "r", '.') %4°

%% Model curves
plot(alpha*180/pi, pacejka(P_R20, L, 250, 0, alpha));
plot(alpha*180/pi, pacejka(P_R20, L, 250, 2, alpha));
plot(alpha*180/pi, pacejka(P_R20, L, 250, 4, alpha));

plot(alpha*180/pi, pacejka(P_R20, L, 150, 0, alpha));
plot(alpha*180/pi, pacejka(P_R20, L, 150, 2, alpha));
plot(alpha*180/pi, pacejka(P_R20, L, 150, 4, alpha));

plot(alpha*180/pi, pacejka(P_R20, L, 50, 0, alpha));
plot(alpha*180/pi, pacejka(P_R20, L, 50, 2, alpha));
plot(alpha*180/pi, pacejka(P_R20, L, 50, 4, alpha));

title("R20 Run 6 vs Model (12 psi, IA Sweep)")
legend("250 0° (sweep 2)", "250 2°", "250 4°")
ylabel("FY (lbf)")
xlabel("SA (deg)")
grid on
hold off