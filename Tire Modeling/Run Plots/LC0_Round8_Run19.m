close all;   
clear;        
clc; 

addpath(genpath("../../Tire Modeling"))

S = load('Data Files/A1965run19'); %load run data as struct
S_raw = load('Data Files/A1965raw19'); %load raw data as struct
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