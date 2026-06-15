clear all
clc
close all

%powertrain parameters
T = [29.8,36.6,44.7,50.2,52.9,48.8,40.7,36.6,29.8,23]; %Torque [N*m]
N = [5,6,7,8,9,10,11,12,13,14]*1000; %RPM
% Calculate the power using the formula P = T * N / 9550
P = T .* N / 9.55; % Power [W]
r = [2.75, 2, 1.667, 1.444, 1.304, 1.208]; %gear ratios 1-5
r_p = 2.111; %primary gear reduction
r_f = 32/11; %final drive ratio
t_shift = 0.1; %shift time [s]

%vehicle setup parameters
h = 0.3175*1.1; %cg height [m]
TL = 1.5367; %track length [m]
m = 272; %mass with driver [kg]
weightDistF = 0.485; %percent of static weight on front axle
R_tire = 0.2032; %nominal tire radius [m]
R_loaded = 0.193; %loaded tire radius [m]
b = TL*(1-weightDistF); %x distance from front axle to cg [m]
mu = 1.5; %approximate loaded longitidunal friction coefficient
g = 9.81;

%aero parameters\
rho = 1.189; % air density [kg/m^3]
F_a_drag = @(v, CDA) 0.5*rho*CDA*v^2;
F_a_downforce = @(v, CLA) 0.5*rho*CLA*v^2;

%rolling resistance
F_rr = m*g*0.02;

SR = 0.1; %assumed slip ratio
%define rpm function. Inputs speed V and gear n, outputs engine RPM
rpm = @(V, n) (SR*V+V)/R_tire*(r(n)*r_p*r_f)*60/(2*pi); 


%% Base Case (No DRS)
CDA = 1.6; % CD*frontal area [m^2]
CLA = 3.6; % CL*frontal area [m^2]
aeroBalance_R = 0.56; %percent rear aerobalance

D = 75; %drag strip length [m]
gear = 1; %starting gear
x_i = 0; %initialize position
v_i = 0; %initialize velocity
t_i = 0; %initialize time vector
dt = 0.001; %time step
%F_max = (mu*m*g*b/TL + mu*F_a_downforce(v)*aeroBalance_R)/(1-h/TL*mu); %traction limit
i = 0;

[T_max, i_Tmax] = max(T); %max torque value and index
rpm_Tmax = N(i_Tmax); %rpm which peak torque occurs
v_release = rpm_Tmax/(r(gear)*r_p*r_f)*2*pi/60*R_tire/(1+SR); %ideal speed for clutch release
while x_i < D
    F_max = (mu*m*g*b/TL + mu*F_a_downforce(v_i, CLA)*aeroBalance_R)/(1-h/TL*mu); %traction limit
    if v_i < v_release
        rpm_i = rpm_Tmax;
    else
        rpm_i = rpm(v_i, gear);
        %check if next gear makes more power
        if gear < 6
        if rpm_i*interp1(N,T,rpm_i) < rpm(v_i, gear+1)*interp1(N,T,rpm(v_i, gear+1))
            gear = gear + 1; %shift to next gear
            t_i = t_i + t_shift; %add shift time
            x_i = x_i + v_i*t_shift;
            rpm_i = rpm(v_i, gear); %update rpm for new gear
        end
        end
    end
    T_i = interp1(N,T,rpm_i);
    F_i = T_i*(r(gear)*r_p*r_f)/R_loaded;
    if F_i > F_max
        F_i = F_max;
    end
    F_accel = F_i - F_a_drag(v_i, CDA) - F_rr; %force contributing to acceleration
    a = F_accel / (1.05*m); %instantaneous acceleration (5% adjustment for rotating mass)
    v_i = v_i + a*dt; %update velocity
    x_i = x_i + v_i * dt; % update position
    t_i = t_i + dt; % update time vector
    i = i + 1; %increase index
    g_base(i) = a/9.81;
    v_base(i) = v_i;
    x_base(i) = x_i;
    t_base(i) = t_i;
end

%% DRS Case
CDA = 1.3; % CD*frontal area [m^2]
CLA = 3.2; % CL*frontal area [m^2]
aeroBalance_R = 0.48; %percent rear aerobalance

D = 75; %drag strip length [m]
gear = 1; %starting gear
x_i = 0; %initialize position
v_i = 0; %initialize velocity
t_i = 0; %initialize time vector
dt = 0.001; %time step
%F_max = (mu*m*g*b/TL + mu*F_a_downforce(v)*aeroBalance_R)/(1-h/TL*mu); %traction limit
i = 0;

[T_max, i_Tmax] = max(T); %max torque value and index
rpm_Tmax = N(i_Tmax); %rpm which peak torque occurs
v_release = rpm_Tmax/(r(gear)*r_p*r_f)*2*pi/60*R_tire/(1+SR); %ideal speed for clutch release
while x_i < D
    F_max = (mu*m*g*b/TL + mu*F_a_downforce(v_i, CLA)*aeroBalance_R)/(1-h/TL*mu); %traction limit
    if v_i < v_release
        rpm_i = rpm_Tmax;
    else
        rpm_i = rpm(v_i, gear);
        %check if next gear makes more power
        if gear < 6
        if rpm_i*interp1(N,T,rpm_i) < rpm(v_i, gear+1)*interp1(N,T,rpm(v_i, gear+1))
            gear = gear + 1; %shift to next gear
            t_i = t_i + t_shift; %add shift time
            x_i = x_i + v_i*t_shift;
            rpm_i = rpm(v_i, gear); %update rpm for new gear
        end
        end
    end
    T_i = interp1(N,T,rpm_i);
    F_i = T_i*(r(gear)*r_p*r_f)/R_loaded;
    if F_i > F_max
        F_i = F_max;
    end
    F_accel = F_i - F_a_drag(v_i, CDA) - F_rr; %force contributing to acceleration
    a = F_accel / (1.05*m); %instantaneous acceleration (5% adjustment for rotating mass)
    v_i = v_i + a*dt; %update velocity
    x_i = x_i + v_i * dt; % update position
    t_i = t_i + dt; % update time vector
    i = i + 1; %increase index
    g_DRS(i) = a/9.81;
    v_DRS(i) = v_i;
    x_DRS(i) = x_i;
    t_DRS(i) = t_i;
end
%% Figure 1: Direct x, v, a comparison
figure;

%Position
subplot(3,1,1)
plot(t_base, x_base, 'LineWidth', 1.5);
hold on
plot(t_DRS, x_DRS, 'LineWidth', 1.5);
ylabel('Position (m)')
title('Base vs DRS Case Performance')
legend('Base Case','DRS Case','Location','best')
grid on

%Velocity
subplot(3,1,2)
plot(t_base, v_base, 'LineWidth', 1.5);
hold on
plot(t_DRS, v_DRS, 'LineWidth', 1.5);
ylabel('Velocity (m/s)')
grid on

%Acceleration (g)
subplot(3,1,3)
plot(t_base, g_base, 'LineWidth', 1.5);
hold on
plot(t_DRS, g_DRS, 'LineWidth', 1.5);
xlabel('Time (s)')
ylabel('Acceleration (g)')
grid on

%% Figure 2: delta x, v, a comparison
dx = x_DRS - interp1(t_base, x_base, t_DRS);
dv = v_DRS - interp1(t_base, v_base, t_DRS);
dg = g_DRS - interp1(t_base, g_base, t_DRS);

figure
subplot(3,1,1)
hold on
title('DRS vs Base Case Delta Performance (DRS - Base)')
plot(t_DRS, dx, 'LineWidth', 1.5)
ylabel('\Delta Position (m)')
grid on

subplot(3,1,2)
plot(t_DRS, dv, 'LineWidth', 1.5)
ylabel('\Delta Velocity (m/s)')
grid on

subplot(3,1,3)
plot(t_DRS, dg, 'LineWidth', 1.5)
xlabel('Time (s)')
ylabel('\Delta Accel (g)')
grid on

