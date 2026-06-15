clear all
clc
close all
load Mock_Endurance_Klot_050925.mat

%% Orient & compress data
g = 9.81;
theta = acos(mean(az,"omitnan")/g);
ax_flat = ax*cos(theta)-az*sin(theta);
ay_flat = ay;

N = 30; % number of samples to average (compression factor)

% Trim to make length divisible by N
ax_flat = ax_flat(1:floor(length(ax_flat)/N)*N);
ay_flat = ay_flat(1:floor(length(ay_flat)/N)*N);

% Reshape and average
ax_avg = mean(reshape(ax_flat, N, []), 1)'; % Result is a column vector
ay_avg = mean(reshape(ay_flat, N, []), 1)'; % Result is a column vector

%% Plot

figure;
plot(ay_avg/g,ax_avg/g, ".", 'MarkerSize', 12)
grid on
xlabel("y-direction G-force")
ylabel("x-direction G-force")
title("Mock Endurance K-lot 5/9/25")