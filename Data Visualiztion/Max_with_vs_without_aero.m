clear all
clc
close all
addpath(genpath("Data Visualization"))
load Max_Aero_051025.mat
load Max_NoAero_051025.mat

%% Orient & compress data
g = 9.81;
theta = acos(mean(az_aero,"omitnan")/g);
ax_aero_flat = ax_aero*cos(theta)-az_aero*sin(theta);
ay_aero_flat = ay_aero;

ax_NoAero_flat = ax_NoAero*cos(theta)-az_NoAero*sin(theta);
ay_NoAero_flat = ay_NoAero;

N = 25; % number of samples to average (compression factor)

% Trim to make length divisible by N
ax_aero_flat = ax_aero_flat(1:floor(length(ax_aero_flat)/N)*N);
ay_aero_flat = ay_aero_flat(1:floor(length(ay_aero_flat)/N)*N);

ax_NoAero_flat = ax_NoAero_flat(1:floor(length(ax_NoAero_flat)/N)*N);
ay_NoAero_flat = ay_NoAero_flat(1:floor(length(ay_NoAero_flat)/N)*N);

% Reshape and average
ax_aero_avg = mean(reshape(ax_aero_flat, N, []), 1)'; % Result is a column vector
ay_aero_avg = mean(reshape(ay_aero_flat, N, []), 1)'; % Result is a column vector

ax_NoAero_avg = mean(reshape(ax_NoAero_flat, N, []), 1)'; % Result is a column vector
ay_NoAero_avg = mean(reshape(ay_NoAero_flat, N, []), 1)'; % Result is a column vector
%% Outlier Rejection
% Combine ax and ay into 2D arrays
aero_data = [ax_aero_avg, ay_aero_avg];
NoAero_data = [ax_NoAero_avg, ay_NoAero_avg];

% Compute z-scores for each column
z_scores_aero = (aero_data - mean(aero_data, 'omitnan')) ./ std(aero_data, 'omitnan');
z_scores_NoAero = (NoAero_data - mean(NoAero_data, 'omitnan')) ./ std(NoAero_data, 'omitnan');

% Threshold for identifying outliers
z_thresh = 3;

% Create masks: reject rows where any component exceeds threshold
valid_idx_aero = all(abs(z_scores_aero) < z_thresh, 2);
valid_idx_NoAero = all(abs(z_scores_NoAero) < z_thresh, 2);

% Apply masks to both ax and ay
ax_aero_avg = ax_aero_avg(valid_idx_aero);
ay_aero_avg = ay_aero_avg(valid_idx_aero);

ax_NoAero_avg = ax_NoAero_avg(valid_idx_NoAero);
ay_NoAero_avg = ay_NoAero_avg(valid_idx_NoAero);

%% Plot

figure;
plot(ay_aero_avg/g,ax_aero_avg/g, ".", 'MarkerSize', 12)
hold on
plot(ay_NoAero_avg/g,ax_NoAero_avg/g, ".", 'MarkerSize', 12)
grid on
xlabel("Lateral G-force")
ylabel("Longitudinal G-force")
title("Max With vs Without Aero")
legend("With Aero","No Aero")
hold off