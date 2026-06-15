clear all
clc
close all

addpath(genpath("Tire Modeling"))

%% Background
%This script is a tool for estimating the steady-state cornering balance
%for a given setup. The primary tuning items of interest are weight
%distribution, aero balance, spring rates, roll centers. It also accounts
%for static toe/camber and ackerman geometry but these are secondary tuning
%paremeters.

%It solves for slip angles using a pure cornering tire model
%Add the end there are also steering force calculations

%% VEHICLE SETUP
%all units in lbs, in (unless specified)

weightDistF = 0.495; %percent of weight on front axle
weightDistL = 0.51; %percent of weight on left
DFDistF = 0.46;  %accounts for moment created by drag force
W_tot = 610; %weight of car and driver
m_tot = W_tot/32.2;
m_uf = 37.5/32.2;  %unsprung front mass
m_ur = 40.5/32.2;  %unsprung rear mass
m_s = W_tot/32.2 - m_uf - m_ur;   %sprung mass
TL = 60.5; %track length
TF = 48; %front track
TR = 48; 
toeF = 0; %static front toe (deg) (wheel plane to centerline)
toeR = 0;
camberF = -1.25; %front static camber (deg)
camberR = -1.25;
castor = 4; %(deg) used to calculate dynamic camber
KPI = 7.6; %(deg) used to calclate dynamic camber
r_l = 7.875; %tire loaded radius
sprung_z = 12.35; %sprung mass CG height
unsprung_z = r_l; %unspring mass approximatly at tire center
CG_z = (sprung_z*m_s+unsprung_z*(m_uf+m_ur))/m_tot; %total CG height

a_x = 0; %to be used once combined tire model is built
a_y = 1*32.2; %array of lateral accelerations to be evaluated
r_corner = 12;   %radius of corner in m, measured from vehicle centerline
delta1 = atan(TL./(r_corner/0.0254+TF/2))*180/pi; %outside front tire toe angle
delta2 = -delta1*(1+0.002079275*delta1) + 2*toeF; %inside front tire toe angle
delta2Ackerman = -atan(TL./(r_corner/0.0254-TF/2))*180/pi; %inside toe angle for 100% ackerman
toeEff = (delta2 - delta2Ackerman)/2; %effective toe on front axle, accounting for ackerman

V = sqrt(r_corner*a_y/32.2*9.81); %velocity in m/s
CL = 3.05;
CLCD = 2;
LF = @(V) 1/2*1.225*V^2*CL*1.08*0.224809;   %downforce, lbf
DF = @(V) LF(V)/CLCD;   %drag force

rc_zf = 2.329; %roll center height front
rc_zr = 2.644; %roll center height rear

kRoll_ubar = 0; %front ARB stiffness in N*m/deg
kRoll_tbar = 317.6;   %(MF12 = 550) 200-400 target

kWheel_f = 307.5; %wheel rate lbf/in %370, 307.5
kWheel_r = 272.5; %327, 272.5

kRoll_f_W = kWheel_f*TF^2*tan(pi/180)/2*0.113;  %roll gradient from coilovers in N*m/deg
kRoll_r_W = kWheel_r*TR^2*tan(pi/180)/2*0.113;

kRoll_f = kRoll_f_W + kRoll_ubar;  %total roll gradients, Nm/deg
kRoll_r = kRoll_r_W + kRoll_tbar;

%camber change from steering
IA1 = camberF - castor*sind(delta1) + KPI*(1-cosd(delta1)); %front outside
IA2 = camberF - castor*sind(delta2) + KPI*(1-cosd(delta2)); %front inside

%camber change from heave/roll
rollCouple = a_y*m_s*(sprung_z-(rc_zr+rc_zf)/2);
thetaRoll = rollCouple*0.113/(kRoll_f+kRoll_r); %degrees of kinematic body roll
heave = LF(V)/(kWheel_f+kWheel_r); %positive = compression
IA1 = IA1 - 1.12*heave + 0.531*thetaRoll;  %front outside camber
IA2 = IA2 - 1.12*heave - 0.531*thetaRoll;  %front inside camber
IA3 = camberR - 0.972*heave + 0.593*thetaRoll; %rear outside camber
IA4 = camberR - 0.972*heave - 0.593*thetaRoll; %rear inside camber

%% CALCULATE INDIVIDUAL WHEEL LOADS
%all deltaWs represent weight transfered from inside to outside
%first calculate static loads, with downforce
W_static_rR = (W_tot*(1-weightDistF)+LF(V)*(1-DFDistF))*(1-weightDistL);
W_static_rL = (W_tot*(1-weightDistF)+LF(V)*(1-DFDistF))*(weightDistL);
W_static_fR = (W_tot*(weightDistF)+LF(V)*(DFDistF))*(1-weightDistL);
W_static_fL = (W_tot*(weightDistF)+LF(V)*(DFDistF))*(weightDistL);

%calculate unsprung load transfer
deltaW_uf = a_y*m_uf*r_l/TR;
deltaW_ur = a_y*m_ur*r_l/TF;

%sprung mass load transfer through suspension links
deltaW_sff = (a_y*m_s*weightDistF)*(rc_zf/TF);
deltaW_sfr = (a_y*m_s*(1-weightDistF))*(rc_zr/TR);


%jacking from steering geometry, negative indicates outside to inside transfer
deltaW_jacking = 7/20*delta1/2;   %function of steering angle, measured 7 at 20 deg steer
deltaW_f_jacking = -deltaW_jacking;
deltaW_r_jacking = deltaW_jacking;

%longitudinal load transfer
deltaLong = a_x*m_tot*CG_z/TL;

%Calculate individual wheel loads [fL, fR; rL, rR]

wheelLoads = zeros(2,2);
wheelLoads(1,1) = W_static_fL + deltaW_sff + deltaW_uf + deltaW_f_jacking - deltaLong/2;
wheelLoads(1,2) = W_static_fR - deltaW_sff - deltaW_uf - deltaW_f_jacking - deltaLong/2;
wheelLoads(2,1) = W_static_rL + deltaW_sfr + deltaW_ur + deltaW_r_jacking + deltaLong/2;
wheelLoads(2,2) = W_static_rR - deltaW_sfr - deltaW_ur - deltaW_r_jacking + deltaLong/2;
frontLoad = wheelLoads(1,1) + wheelLoads(1,2);
rearLoad = wheelLoads(2,1) + wheelLoads(2,2);
totalLoad = frontLoad + rearLoad;
leftLoad = wheelLoads(1,1) + wheelLoads(2,1);
rightLoad = totalLoad - leftLoad;

deltaW_rear = wheelLoads(2,1) - wheelLoads(2,2);
deltaW_front = wheelLoads(1,1) - wheelLoads(1,2);

%% Chassis torsion modifier
h_r = 0.376;
h_f = 0.254;
k_c = linspace(50,3000,1000); % Example stiffness values for the chassis
deltaW_c = zeros(1, length(k_c));
deltaW_f = zeros(1, length(k_c));
deltaW_r = zeros(1, length(k_c));

for i = 1:length(k_c)
    A = [kRoll_f+k_c(i), -k_c(i); -k_c(i), kRoll_r+k_c(i)];
    b = [m_uf*32.2/2.2*a_y/32.2*9.81*(h_f-rc_zf)*25.4/1000;m_ur*32.2/2.2*a_y/32.2*9.81*(h_r-rc_zr)*25.4/1000];
    x = A \ b; % Solve for the load distribution
    deltaW_c(i) = k_c(i)*(x(1)-x(2));
    deltaW_f(i) = kRoll_f*x(1);
    deltaW_r(i) = kRoll_r*x(2);
end

