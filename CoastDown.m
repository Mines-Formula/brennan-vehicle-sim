clear all
clc
close all

CD = 1.4;
CL = 3;
m_tot = 272.7; %kg
A = 1.2; %m^2
rho = 1; %kg/m^3


F_D = @(V) 0.5*CD*rho*A*V^2;
F_L = @(V) 0.5*CL*rho*A*V^2;
F_rr = @(V) 0.02*(m_tot+F_L(V));

%% 

v0 = 11/2.237; %initial velocity (m/s)
V = v0;
dt = 0.001;
a = 0;
D = 0;
i = 1;
while V(i) > 3
    a(i) = (F_D(V(i))+F_rr(V(i)))/m_tot;
    V(i+1) = V(i) - a(i)*dt; % Update velocity
    D(i+1) = D(i) + V(i) * dt; % Update distance covered
    i = i + 1;
end

t = linspace(0,length(V),length(V))*dt;

figure;
plot(t,V)
xlabel("Time Elapsed (s)")
ylabel("Velocity (m/s)")
title("Coast Down from 60 MPH")
grid on

figure;
plot(D,V)
xlabel("Distance (m)")
ylabel("Velocity (m/s)")
title("Coast Down from 60 MPH")
grid on