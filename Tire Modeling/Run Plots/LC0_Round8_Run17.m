%Round 8 Run 17 is with 16x7.5-10 LC0s mounted on a 7 in wide rim
%The first section of this run tests initial vertical stiffness
%Remainder of the run tests transient response to step steers

%See "LC0 Round 8 Calculations.xlsx" for more info

close all;   
clear;        
clc; 

addpath(genpath("../../Tire Modeling"))

S = load('Data Files/A1965run17'); %load run data as struct
S_raw = load('Data Files/A1965raw17'); %load raw data as struct
S = rmfield(S, {'channel', 'source', 'testid', 'tireid'}); %remove these fields
S_raw = rmfield(S_raw, {'channel', 'source', 'testid', 'tireid'}); %remove these fields
T = struct2table(S); %covert struct 2 table
T_raw = struct2table(S_raw); %covert struct 2 table
N = 20; %running average factor
T_sm = varfun(@(x) movmean(x,N), T); %smooth data with running average
T_sm.Properties.VariableNames = T.Properties.VariableNames; %set variable names T_sm
T_raw_sm = varfun(@(x) movmean(x,N), T_raw); %smooth data with running average
T_raw_sm.Properties.VariableNames = T_raw.Properties.VariableNames; %set variable names T_sm

i = linspace(1,length(T_sm.ET),length(T_sm.ET)); %index vector
i_raw = linspace(1,length(T_raw_sm.ET),length(T_raw_sm.ET)); %index vector

%plot control parameters for full run data
figure;
scatter(i,T_sm.SA, "r", '.')
title("Round 8, Run 18 Data")
hold on
scatter(i,T_sm.IA, "b", '.')
scatter(i,T_sm.FZ, "k", '.')
scatter(i,T_sm.P, "y", '.')
scatter(i, T_sm.V, "m", '.')
scatter(i, T_sm.TSTI, "r", '.')
scatter(i, T_sm.TSTC, "g", '.')
scatter(i, T_sm.TSTO, "b", '.')
legend("SA (deg)", "IA (deg)", "FZ (lbf)", "P (psi)", "V (mph)", "TSTI (F)", "TSTC (F)", "TSTO (F)")
grid on
hold off
%plot control parameters for full run data, time series
figure;
scatter(T.ET,T_sm.SA, "r", '.')
title("Round 8, Run 18 Data Time Series")
hold on
scatter(T.ET,T_sm.IA, "b", '.')
scatter(T.ET,T_sm.FZ, "k", '.')
scatter(T.ET,T_sm.P, "y", '.')
scatter(T.ET, T_sm.V, "m", '.')
scatter(T.ET, T_sm.TSTI, "r", '.')
scatter(T.ET, T_sm.TSTC, "g", '.')
scatter(T.ET, T_sm.TSTO, "b", '.')
legend("SA (deg)", "IA (deg)", "FZ (lbf)", "P (psi)", "V (mph)", "TSTI (F)", "TSTC (F)", "TSTO (F)")
grid on
hold off

%plot control parameters for full raw data
figure;
scatter(T_raw.ET,T_raw_sm.SA, "r", '.')
title("Round 8, Run 18 Raw Data")
hold on
scatter(T_raw.ET,T_raw_sm.IA, "b", '.')
scatter(T_raw.ET,T_raw_sm.FZ, "k", '.')
scatter(T_raw.ET,T_raw_sm.P, "y", '.')
scatter(T_raw.ET, T_raw_sm.V, "m", '.')
scatter(T_raw.ET, T_raw_sm.TSTI, "r", '.')
scatter(T_raw.ET, T_raw_sm.TSTC, "g", '.')
scatter(T_raw.ET, T_raw_sm.TSTO, "b", '.')
legend("SA (deg)", "IA (deg)", "FZ (lbf)", "P (psi)", "V (mph)", "TSTI (F)", "TSTC (F)", "TSTO (F)")
grid on
hold off

%% Initial Vertical Siffness Tests @12psi

i1 = 1; %start index of interest
i2 = 1470; %end index of interest
%plot loaded radius with FZ, IA, P, and V
figure;
yyaxis left
title("LC0 Initial Vertical Stiffness Tests")
scatter(i(i1:i2),T_sm.IA(i1:i2), "b", '.')
hold on
scatter(i(i1:i2),T_sm.FZ(i1:i2), "k", '.')
scatter(i(i1:i2),T_sm.P(i1:i2), "y", '.')
scatter(i(i1:i2), T_sm.V(i1:i2), "m", '.')
yyaxis right
ylabel("Tire Loaded Radius (in)")
scatter(i(i1:i2), T_sm.RL(i1:i2), "g", '.')
legend("IA (deg)", "FZ (lbf)", "P (psi)", "V (mph)", "RL")
grid on
hold off

%% Transient Step Steer Tests

%12psi, 0IA, -1SA
%to evauate other setups, change the indicies below
i_50 = [2000,3000];
i_150 = i_50 + 2100;
i_250 = i_150 + 2100;

figure;
hold on
title("Example Transient Test Visualization")
yyaxis left
scatter(T_sm.ET(i_50(1):i_50(2)), T_sm.MZ(i_50(1):i_50(2)), 'b', '.')
scatter(T_sm.ET(i_50(1):i_50(2)), T_sm.FY(i_50(1):i_50(2)), 'r', '.')
yyaxis right
ylabel("Speed (MPH)")
scatter(T_sm.ET(i_50(1):i_50(2)), T_sm.V(i_50(1):i_50(2)), 'k', '.')
xlabel("Elapsed Time (s)");
grid on
hold off
legend("MZ (lbf-ft", "FY (lbf)", "V (MPH)")
