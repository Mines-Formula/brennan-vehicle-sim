clear all
clc
close all
load Max_Skidpad_041925_data

%% Orient & compress data
g = 9.81;
theta = acos(mean(az)/g);
ax_flat = ax*cos(theta)-az*sin(theta);
ay_flat = ay;

N = 20; % number of samples to average (compression factor)

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
title("Skidpad Test Runs 4-19-25")