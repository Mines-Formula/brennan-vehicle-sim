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

weightDistF = 0.49; %percent of weight on front axle
weightDistL = 0.51; %percent of weight on left
DFDistF = 0.46;  %accounts for moment created by drag force
W_tot = 580; %weight of car and driver
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

a_x = zeros(1,1000); %to be used once combined tire model is built
a_y = linspace(1,1.66,1000)*32.2; %array of lateral accelerations to be evaluated
r_corner = 10;   %radius of corner in m, measured from vehicle centerline
delta1 = atan(TL./(r_corner/0.0254+TF/2))*180/pi; %outside front tire toe angle
delta2 = -delta1*(1+0.002079275*delta1) + 2*toeF; %inside front tire toe angle
delta2Ackerman = -atan(TL./(r_corner/0.0254-TF/2))*180/pi; %inside toe angle for 100% ackerman
toeEff = (delta2 - delta2Ackerman)/2; %effective toe on front axle, accounting for ackerman

V = sqrt(r_corner.*a_y/32.2*9.81); %velocity in m/s
CL = 3.05;
CLCD = 2;
LF = @(V) 1/2*1.225*V.^2*CL*1.08*0.224809;   %downforce, lbf
DF = @(V) LF(V)/CLCD;   %drag force

rc_zf = 2.329; %roll center height front
rc_zr = 2.644; %roll center height rear

kRoll_ubar = 0; %front ARB stiffness in N*m/deg
kRoll_tbar = 317.6;   %(MF12 = 550) 200-400 target

kWheel_f = 307.5; %wheel rate lbf/in %370, 307.5
kWheel_r = 272.5; %327, 272.5

kRoll_f_W = kWheel_f.*TF.^2*tan(pi/180)/2*0.113;  %roll gradient from coilovers in N*m/deg
kRoll_r_W = kWheel_r.*TR.^2*tan(pi/180)/2*0.113;

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
deltaW_sff = (a_y*m_s*weightDistF)*(rc_zf./TF);
deltaW_sfr = (a_y*m_s*(1-weightDistF))*(rc_zr./TR);

%sprung mass through springs resisting roll couple
deltaW_scf = kRoll_f./(kRoll_r+kRoll_f)*rollCouple./TF;
deltaW_scr = kRoll_r./(kRoll_r+kRoll_f)*rollCouple./TR;

%jacking from steering geometry, negative indicates outside to inside transfer
deltaW_jacking = 7/20*delta1/2;   %function of steering angle, measured 7 at 20 deg steer
deltaW_f_jacking = -deltaW_jacking;
deltaW_r_jacking = deltaW_jacking;

%longitudinal load transfer
deltaLong = a_x*m_tot*CG_z/TL;

%Calculate individual wheel loads [fL, fR; rL, rR]

wheelLoads = cell(2,2);
wheelLoads{1,1} = W_static_fL + deltaW_scf + deltaW_sff + deltaW_uf + deltaW_f_jacking - deltaLong/2;
wheelLoads{1,2} = W_static_fR - deltaW_scf - deltaW_sff - deltaW_uf - deltaW_f_jacking - deltaLong/2;
wheelLoads{2,1} = W_static_rL + deltaW_scr + deltaW_sfr + deltaW_ur + deltaW_r_jacking + deltaLong/2;
wheelLoads{2,2} = W_static_rR - deltaW_scr - deltaW_sfr - deltaW_ur - deltaW_r_jacking + deltaLong/2;
frontLoad = wheelLoads{1,1} + wheelLoads{1,2};
rearLoad = wheelLoads{2,1} + wheelLoads{2,2};
totalLoad = frontLoad + rearLoad;
leftLoad = wheelLoads{1,1} + wheelLoads{2,1};
rightLoad = totalLoad - leftLoad;

deltaW_rear = wheelLoads{2,1} - wheelLoads{2,2};
deltaW_front = wheelLoads{1,1} - wheelLoads{1,2};

%% CALCULATE BASIC LATERAL GRIPS AT EACH AXLE
Fy_tot = m_tot*a_y;  %total lateral grip required, simple F=ma
Fy_front = Fy_tot*weightDistF;  %initial estimates for lateral forces at each axle
Fy_rear = Fy_tot*(1-weightDistF);
%% INITIAL SA ESTIMATE
P=[250,1.4,2.4,-0.25,3,-0.1,-1.5,0,0,-30.5,1.15,1,0,0,-0.128,0,0,0,1.43]; %pacejka coeffs
L=[0.62,1,1,1,1,1,1,1]; %scaling factors
n = length(Fy_front);
frontSA = zeros(1,n); 
rearSA = zeros(1,n);

for i = 1:n
    %findSlip using bisection to solve for SA to satisfy inputs
    frontSA(i) = findSlip(wheelLoads{1,1}(i),frontLoad(i),Fy_front(i),IA1(i),IA2(i),toeEff,P,L)*pi/180;
    rearSA(i) = findSlip(wheelLoads{2,1}(i),rearLoad(i),Fy_rear(i),IA3(i),IA4(i),toeR,P,L)*pi/180;
end
%% ADJUST LATERAL GRIP BASED ON DRIVE/BRAKE (TRACTION CIRCLE EFFECT)
n = length(a_x);
Fx_rear = zeros(1,n);
Fx_front = zeros(1,n);
parasiticDrag = a_y*m_tot.*sin(rearSA) + 0.5*a_y*m_tot.*sin(frontSA-rearSA) + 0.02*W_tot; %second term is usually insignificant
for i = 1:n
    if a_x(i) < 0 %braking condition, 30/70 bias
        Fx_rear(i) = (a_x(i)*m_tot + parasiticDrag(i) + DF(V(i)))*0.3;
        Fx_front(i) = (a_x(i)*m_tot + parasiticDrag(i) + DF(V(i)))*0.7;
    else %aceleration or constant speed condition, RWD
        Fx_rear(i) = a_x(i)*m_tot + parasiticDrag(i) + DF(V(i));
    end
end
Fy_front = sqrt((Fy_front).^2 + (Fx_front).^2);     %new adjusted required lateral grips
Fy_rear = sqrt((Fy_rear).^2 + (Fx_rear).^2);
%% ROLLING RESISTANCE UNDERSTEER MOMENT
Mu = 0.03*(leftLoad - rightLoad).*TF/2; %understeer moment from drag force (~3% of normal load)
deltaF_rollingResistance = Mu./TL;
%% UPDATE ESTIMATE OF SAs
for i = 1:n
    frontSA(i) = findSlip(wheelLoads{1,1}(i),frontLoad(i),Fy_front(i),IA1(i),IA2(i),toeEff,P,L)*pi/180;
    rearSA(i) = findSlip(wheelLoads{2,1}(i),rearLoad(i),Fy_rear(i),IA3(i),IA4(i),toeR,P,L)*pi/180;
end
%% MZs/SELF ALIGNING TORQUE (UNDERSTEER MOMENT)
%uses MZ tire model for pure cornering
PM=[250,2.5,0.16,0.12,0,-0.065,-0.8,0,0,4.8,1.8,0,0,0,0.2,-0.01,0,0.4,0,-0.045,250];
LMZ=[0.62,1,1,1,1,1,1,1];
MZ_fo = zeros(1,n);
MZ_fi = zeros(1,n);
MZ_ro = zeros(1,n);
MZ_ri = zeros(1,n);
for i = 1:n
MZ_fo(i) = pacejkaMZ(PM,LMZ,wheelLoads{1,1}(i),IA1(i)*pi/180,frontSA(i))*12;
MZ_fi(i) = pacejkaMZ(PM,LMZ,wheelLoads{1,2}(i),-IA2(i)*pi/180,frontSA(i))*12;
MZ_ro(i) = pacejkaMZ(PM,LMZ,wheelLoads{2,1}(i),IA3(i)*pi/180,rearSA(i))*12;
MZ_ri(i) = pacejkaMZ(PM,LMZ,wheelLoads{2,2}(i),-IA4(i)*pi/180,rearSA(i))*12;
end

Mu = MZ_fi+MZ_fo+MZ_ro+MZ_ri; %estimate each corner using SA, Fz, and camber angle
deltaF_selfAllign = Mu./TL;
%% INDUCED TIRE DRAG (UNDERSTEER MOMENT)
%this section could be improved to account for moment created due to
%induced tire drag at each corner (account for static toe/ackerman
%settings)
%estimate SA at each corner
SA1 = frontSA + toeEff*pi/180;
SA2 = frontSA - toeEff*pi/180;
SA3 = rearSA + toeR*pi/180;
SA4 = rearSA - toeR*pi/180;
%understeer moment due to difference in front and rear SA
Mu =  (wheelLoads{1,1}-wheelLoads{1,2}).*a_y/32.2.*sin(frontSA - rearSA).*TF/2;
%understeer moment due to difference in front inner/outer induced tire drag
%Mu = Mu - (pacejka(P,L,wheelLoads{1,1},0,SA1).*sin(SA1)-pacejka(P,L,wheelLoads{1,2},0,SA2).*sin(SA2))*TF/2;
%understeer moment due to difference in rear inner/outer induced tire drag
%Mu = Mu - (pacejka(P,L,wheelLoads{2,1},0,SA3).*sin(SA3)-pacejka(P,L,wheelLoads{2,2},0,SA4).*sin(SA4))*TR/2;
deltaF_inducedDrag = Mu./TF; 

%if alphaR > alphaF, contributes to additional oversteer (and vice versa)
%also sensitive to toe changes (i think?)
%% COMPUTE REQUIRED LATERAL GRIPS
Fy_front = Fy_front + deltaF_rollingResistance + deltaF_selfAllign + deltaF_inducedDrag;
Fy_rear = Fy_rear - deltaF_rollingResistance - deltaF_selfAllign - deltaF_inducedDrag;

%% PLOT SAs USING TIRE MODEL
%first correct SAs based on understeer moments
for i = 1:n
    frontSA(i) = findSlip(wheelLoads{1,1}(i),frontLoad(i),Fy_front(i),IA1(i),IA2(i),toeEff,P,L);
    rearSA(i) = findSlip(wheelLoads{2,1}(i),rearLoad(i),Fy_rear(i),IA3(i),IA4(i),toeR,P,L);
end

figure;
plot(a_y/32.2, frontSA)
hold on
grid on
plot(a_y/32.2, rearSA)
ylabel("SA (deg)")
xlabel("Cornering g-force")
title("Comparison of Front vs Rear Slip Angle")
legend("Front","Rear");
hold off

figure;
plot(a_y/32.2, Fy_front)
hold on
grid on
plot(a_y/32.2, Fy_rear)
ylabel("Required Lateral Grip (lbf)")
xlabel("Cornering g-force")
legend("Front","Rear");
hold off

figure;
plot(a_y/32.2, wheelLoads{1,1}-wheelLoads{1,2})
hold on
grid on
plot(a_y/32.2, wheelLoads{2,1}-wheelLoads{2,2})
ylabel("Load Transfer (lbf)")
xlabel("Cornering g-force")
legend("Front","Rear");
hold off

figure;
plot(a_y/32.2, wheelLoads{1,2})
hold on
grid on
plot(a_y/32.2, wheelLoads{2,2})
ylabel("Inside Load (lbf)")
xlabel("Cornering g-force")
legend("Front","Rear");
hold off
%% Plot Understeer Gradient
figure;
theta_steer = frontSA - rearSA + (abs(delta1)+abs(delta2))/2;
plot(a_y/32.2, theta_steer)
xlabel("Lateral Acceleration [g]")
ylabel("Steering Angle [deg]")
title("Understeer Gradient for 15m Radius Corner")
%% Steering Forces
d = 0.579; %scrub radius (in)
KPI = 7.6*pi/180; %KPI angle (rad)
theta_steer = 10*pi/180; %steering angle (rad)
castor = 3.94*pi/180; %castor angle (rad)
trail = 0.552; %mechanical trail (in) 
r = 7.875; %tire radius (in)

%moment due to vertical force
M_V = -(wheelLoads{1,1}+wheelLoads{1,2})*d*sin(KPI)*sin(theta_steer)+...
    (wheelLoads{1,1}-wheelLoads{1,2})*d*sin(castor)*sin(theta_steer); 

%moment due to lateral force
M_L = -(Fy_front)*trail;

%monent due to aligning torque
M_AT = -(MZ_fi+MZ_fo)*cos(sqrt(KPI^2+castor^2));

M_tot = M_V + M_L + M_AT;

L_arm = 2.95; %steering arm length (in)
F_rack = M_tot/(L_arm*cos(theta_steer));

r_pinion = 1.25/2; %pinion gear radius (in)
T_column = F_rack*r_pinion; %steering column torque (lbf-in)

D_wheel = 8.5; %steering wheel diameter (in)
F_wheel = -T_column/D_wheel; %steering wheel force in each hand (lbf)

%%Repeat for MF12 trail
d = 0.539; %scrub radius (in)
KPI = 8*pi/180; %KPI angle (rad)
theta_steer = 10*pi/180; %steering angle (rad)
castor = 4*pi/180; %castor angle (rad)
trail = 0.752; %mechanical trail (in) 
r = 7.875; %tire radius (in)

%moment due to vertical force
M_V = -(wheelLoads{1,1}+wheelLoads{1,2})*d*sin(KPI)*sin(theta_steer)+...
    (wheelLoads{1,1}-wheelLoads{1,2})*d*sin(castor)*sin(theta_steer); 

%moment due to lateral force
M_L = -(Fy_front)*trail;

%monent due to aligning torque
M_AT = -(MZ_fi+MZ_fo)*cos(sqrt(KPI^2+castor^2));

M_tot = M_V + M_L + M_AT;

L_arm = 2.95; %steering arm length (in)
F_rack = M_tot/(L_arm*cos(theta_steer));

r_pinion = 1.25/2; %pinion gear radius (in)
T_column1 = F_rack*r_pinion; %steering column torque (lbf-in)

D_wheel = 8.5; %steering wheel diameter (in)
F_wheel1 = -T_column1/D_wheel; %steering wheel force in each hand (lbf)

%%Repeat for MF11 trail
d = 0.77; %scrub radius (in)
KPI = 5.34*pi/180; %KPI angle (rad)
theta_steer = 10*pi/180; %steering angle (rad)
castor = 2*pi/180; %castor angle (rad)
trail = 0.475; %mechanical trail (in) 
r = 7.875; %tire radius (in)

%moment due to vertical force
M_V = -(wheelLoads{1,1}+wheelLoads{1,2})*d*sin(KPI)*sin(theta_steer)+...
    (wheelLoads{1,1}-wheelLoads{1,2})*d*sin(castor)*sin(theta_steer); 

%moment due to lateral force
M_L = -(Fy_front)*trail;

%monent due to aligning torque
M_AT = -(MZ_fi+MZ_fo)*cos(sqrt(KPI^2+castor^2));

M_tot = M_V + M_L + M_AT;

L_arm = 2.95; %steering arm length (in)
F_rack = M_tot/(L_arm*cos(theta_steer));

r_pinion = 1.25/2; %pinion gear radius (in)
T_column2 = F_rack*r_pinion; %steering column torque (lbf-in)

D_wheel = 8.5; %steering wheel diameter (in)
F_wheel2 = -T_column2/D_wheel; %steering wheel force in each hand (lbf)


figure;
plot(a_y/32.2, F_wheel)
hold on
plot(a_y/32.2, F_wheel1)
plot(a_y/32.2, F_wheel2)
xlim([0.8,1.9])
xlabel("Cornering G-force")
ylabel("Steering Force (lbf)")
legend("MF13", "MF12", "MF11")
grid on
title("Steering Force at 10 deg Steering Angle, 8.5 in Wheel")

figure;
plot(a_y/32.2, -T_column)
hold on
plot(a_y/32.2, -T_column1)
plot(a_y/32.2, -T_column2)
xlim([0.8,1.9])
xlabel("Cornering G-force")
ylabel("Column Torque (lbf-in)")
legend("MF13", "MF12", "MF11")
grid on
title("Steering Column Torque at 10 deg Steering Angle")





