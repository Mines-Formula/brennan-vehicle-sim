clear all
clc
close all
load Max_Endurance_MF12.mat

%% Set NaN values = 0
az(isnan(az)) = 0;
ay(isnan(ay)) = 0;
ax(isnan(ax)) = 0;
%% Orient & compress data
%all the code below is straight from chatGPT
% Raw acceleration data
% ax, ay, az should be column vectors of equal length
g = 9.81;
% Step 1: Use the first 500 samples to determine the orientation
sample_indices = 1:500;
g_raw = mean([ax(sample_indices), ay(sample_indices), az(sample_indices)], 1)';

% Normalize the gravity vector
g_raw = g_raw / norm(g_raw);

% Desired gravity vector in car frame (pointing up)
g_target = [0; 0; 1];

% Step 2: Compute rotation matrix to align g_raw with g_target
v = cross(g_raw, g_target);
s = norm(v);
c = dot(g_raw, g_target);

if s ~= 0
    vx = [  0   -v(3)  v(2);
          v(3)   0   -v(1);
         -v(2)  v(1)   0  ];
    R_flatten = eye(3) + vx + vx^2 * ((1 - c)/(s^2));
else
    R_flatten = eye(3); % Already aligned
end

% Step 3: Rotate all data using the flattening rotation
raw_data = [ax, ay, az]';
flattened_data = R_flatten * raw_data;
ax_flat = flattened_data(1, :)';
ay_flat = flattened_data(2, :)';
az_flat = flattened_data(3, :)';

% Step 4: Rotate about Z to align with car's heading
theta = 95*pi/180;
R_z = [cos(theta), -sin(theta), 0;
       sin(theta),  cos(theta), 0;
           0,           0,      1];

rotated_data = R_z * [ax_flat'; ay_flat'; az_flat'];
ax_car = rotated_data(1, :)';
ay_car = rotated_data(2, :)';
az_car = rotated_data(3, :)';

N = 30; % number of samples to average (compression factor)

% Trim to make length divisible by N
ax_car = ax_car(1:floor(length(ax_car)/N)*N);
ay_car = ay_car(1:floor(length(ay_car)/N)*N);

% Reshape and average
ax_avg = mean(reshape(ax_car, N, []), 1)'; % Result is a column vector
ay_avg = mean(reshape(ay_car, N, []), 1)'; % Result is a column vector

%% Plot

figure;
plot(ay_avg/g,ax_avg/g, ".", 'MarkerSize', 12)
grid on
xlabel("y-direction G-force")
ylabel("x-direction G-force")
title("MF12 Endurance - Max")