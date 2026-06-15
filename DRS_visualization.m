clear all
clc
close all

load accel_basecase.mat
load accel_DRS.mat


% Ensure we compare both over the same time vector (the basecase timeline)
t_common = basecase.Time;

% Interpolate DRS data to match basecase time steps
% Use 'linear' interpolation and 'extrap' to handle the very end of the run
dist_drs_interp  = interp1(DRS.Time, DRS.("Distance [m]"), t_common, 'linear', 'extrap');
vel_drs_interp   = interp1(DRS.Time, DRS.("Velocity [m/s]"), t_common, 'linear', 'extrap');
accel_drs_interp = interp1(DRS.Time, DRS.("Acceleration [g]"), t_common, 'linear', 'extrap');

% Calculate Deltas (DRS advantage)
delta_dist  = dist_drs_interp - basecase.("Distance [m]");
delta_vel   = vel_drs_interp  - basecase.("Velocity [m/s]");
delta_accel = accel_drs_interp - basecase.("Acceleration [g]");

% Create Figure
figure('Name', 'DRS vs Basecase Delta Analysis', 'Color', 'w');

% 1. Delta Position
subplot(3,1,1);
plot(t_common, delta_dist, 'g', 'LineWidth', 2);
ylabel('\Delta Distance [m]');
title('Position Advantage (DRS - Base)');
grid on;

% 2. Delta Velocity
subplot(3,1,2);
plot(t_common, delta_vel, 'b', 'LineWidth', 2);
ylabel('\Delta Velocity [m/s]');
title('Velocity Advantage (DRS - Base)');
grid on;

% 3. Delta Acceleration
subplot(3,1,3);
plot(t_common, delta_accel, 'r', 'LineWidth', 2);
xlabel('Time [s]');
ylabel('\Delta Accel [g]');
title('Acceleration Advantage (DRS - Base)');
grid on;

% Global formatting
sgtitle('DRS Performance Benefit Over 75m Sprint');