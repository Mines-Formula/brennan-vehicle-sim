clear all
clc
close all

%below is simulated data from MF13 vehicle balance matlab script
load("full_oversteer") %stiffest ARB, 250lb springs all corners
load("understeer_limit_oversteer") %nominal ARB, 250lb springs all corners
load("full_understeer") %no ARB, 300lb front springs 250lb rear springs
load("30m_radius") %nominal setup 30m radius
load("10m_radius") %nominal setup 15m radius

%% Plot 1: ARB Sweep (15 m radius)

figure;
plot(a_y_1/32.2,theta_steer_1,'k','LineWidth',1.5);
hold on
plot(a_y_2/32.2,theta_steer_2,'k','LineWidth',1.5);
plot(a_y_3/32.2,theta_steer_3,'k','LineWidth',1.5);

% Neutral steer line
yline(5.65,'k--');

% Labels
text(a_y_1(end)/32.2,theta_steer_1(end),'  No Rear ARB','FontSize',16);
text(a_y_2(end)/32.2,theta_steer_2(end),'  Nominal Rear ARB','FontSize',16);
text(a_y_3(end)/32.2,theta_steer_3(end),'  Stiffest Rear ARB','FontSize',16);
text(a_y_3(end)/32.2,theta_steer_3(end),'Neutral Steer','FontSize',14);
text(a_y_3(end)/32.2,theta_steer_3(end),'Limit Oversteer','FontSize',14);
text(a_y_3(end)/32.2,theta_steer_3(end),'Limit Understeer','FontSize',14);

title("Constant Radius (15 m) Understeer Gradient")
xlabel('Lateral Acceleration [g]')
ylabel('Steering Angle [deg]')
grid on


%% Plot 2: Radius Comparison (Normalized Steering Angle)

% Normalize steering angles to start at zero
theta2_norm = theta_steer_2 - theta_steer_2(1);
theta4_norm = theta_steer_4 - theta_steer_4(1);
theta5_norm = theta_steer_5 - theta_steer_5(1);

figure;
plot(a_y_2/32.2,theta2_norm,'k','LineWidth',1.5);
hold on
plot(a_y_4/32.2,theta4_norm,'k','LineWidth',1.5);
plot(a_y_5/32.2,theta5_norm,'k','LineWidth',1.5);

% Neutral steer line
yline(0,'k--')

% Labels at end of lines
text(a_y_2(end)/32.2,theta2_norm(end),'  Nominal Setup','FontSize',10);
text(a_y_4(end)/32.2,theta4_norm(end),'  30 m Radius','FontSize',10);
text(a_y_5(end)/32.2,theta5_norm(end),'  10 m Radius','FontSize',10);
text(a_y_5(end)/32.2,theta5_norm(end),'Neutral Steer','FontSize',10);


title("Constant Radius Comparison (Normalized Steering Angle)")
xlabel('Lateral Acceleration [g]')
ylabel('\Delta Steering Angle [deg]')
grid on