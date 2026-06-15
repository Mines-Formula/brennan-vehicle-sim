%% VEHICLE SETUP
%all units in lbs, in (unless specified)

weightDistF = 0.485;
weightDistF = 0.2;
DFDistF = 0.4;  %accounts for moment created by drag force
W_tot = 610;
m_tot = W_tot/32.2;
m_uf = 37.5/32.2;  %unsprung front mass
m_ur = 40.5/32.2;  %unsprung rear mass
m_s = W_tot/32.2 - m_uf - m_ur;   %sprung mass
TL = 60.5;
TF = 48;
TR = 48;
r_tires = 7.875;
sprung_z = 13.5;
unsprung_z = r_tires;
CG_z = (sprung_z*m_s+unsprung_z*(m_uf+m_ur))/m_tot;

a_x = zeros(1,100);
a_y = linspace(1,1.8,100)*32.2;
r_corner = 10;   %radius of corner in m

V = sqrt(r_corner.*a_y/32.2*9.81); %velocity in m/s


LF = @(V) 150;   %downforce
DF = @(V) LF(V)/2;   %drag force
%crossWind = @() 0; %implement later?

rc_zf = 2.329; %roll center heights
rc_zr = 2.69;

kRoll_ubar = 0; %ARB stiffness in N*m/deg (MF12 = 950)
kRoll_tbar = 300;   %(MF12 = 550) 200-400 target

kWheel_f = 370; %wheel rate lbf/in 300spring 
kWheel_r = 320; 

kTire = 650; %tire stiffness in lbf/in

kRoll_f_W = kWheel_f.*TF.^2*tan(pi/180)/2*0.113;  %roll gradient from coilovers in N*m/deg
kRoll_r_W = kWheel_r.*TR.^2*tan(pi/180)/2*0.113;

kRoll_f = kRoll_f_W + kRoll_ubar;  %total roll gradients, Nm/deg
kRoll_r = kRoll_r_W + kRoll_tbar;
%% CALCULATE INDIVIDUAL WHEEL LOADS
%all deltaWs represent weight transfered from right to left
%first calculate static loads, with DF, drag
W_static_rR = (W_tot*(1-weightDistF)+LF(V)*(1-DFDistF))/2;
W_static_rL = W_static_rR;
W_static_fR = (W_tot*(weightDistF)+LF(V)*(DFDistF))/2;
W_static_fL = W_static_fR;

%calculate unsprung load transfer
deltaW_uf = a_y*m_uf*r_tires/TR;
deltaW_ur = a_y*m_ur*r_tires/TF;

%sprung mass load transfer through suspension links
deltaW_sff = (a_y*m_s*weightDistF)*(rc_zf./TF);
deltaW_sfr = (a_y*m_s*(1-weightDistF))*(rc_zr./TR);

%sprung mass through springs resisting roll couple
rollCouple = a_y*m_s*(sprung_z-(rc_zr+rc_zf)/2);

deltaW_scf = kRoll_f./(kRoll_r+kRoll_f)*rollCouple./TF;
deltaW_scr = kRoll_r./(kRoll_r+kRoll_f)*rollCouple./TR;

%jacking from steering geometry, negative indicates left to right transfer
deltaW_jacking = @(SA) 3;   %function of steering angle, measured 7 at 20 deg steer
thetaSteer = atand(TL/(r_corner*39.37))+5; %steering angle in deg
deltaW_f_jacking = -deltaW_jacking(thetaSteer);
deltaW_r_jacking = deltaW_jacking(thetaSteer);

%longitudinal load transfer
deltaLong = a_x*m_tot*CG_z/TL;

%Calculate individual wheel loads [fL, fR; rL, rR]

wheelLoads = cell(2,2);
wheelLoads{1,1} = W_static_fL + deltaW_scf + deltaW_sff + deltaW_uf + deltaW_f_jacking - deltaLong/2;
wheelLoads{1,2} = W_static_fL - deltaW_scf - deltaW_sff - deltaW_uf - deltaW_f_jacking - deltaLong/2;
wheelLoads{2,1} = W_static_rL + deltaW_scr + deltaW_sfr + deltaW_ur + deltaW_r_jacking + deltaLong/2;
wheelLoads{2,2} = W_static_rL - deltaW_scr - deltaW_sfr - deltaW_ur - deltaW_r_jacking + deltaLong/2;
frontLoad = wheelLoads{1,1} + wheelLoads{1,2};
rearLoad = wheelLoads{2,1} + wheelLoads{2,2};
totalLoad = frontLoad + rearLoad;
leftLoad = wheelLoads{1,1} + wheelLoads{2,1};
rightLoad = totalLoad - leftLoad;



deltaW_rear = wheelLoads{2,1} - wheelLoads{2,2};
deltaW_front = wheelLoads{1,1} - wheelLoads{1,2};

%{
plot(kRoll_ubar, deltaW_rear)
hold on
plot(kRoll_ubar, deltaW_front)
hold off
%}
%% CALCULATE BASIC LATERAL GRIPS AT EACH AXLE
Fy_tot = m_tot*a_y;  %total lateral grip required, simple F=ma
Fy_front = Fy_tot*weightDistF;  %initial estimates for lateral forces at each axle
Fy_rear = Fy_tot*(1-weightDistF);
%% INITIAL SA ESTIMATE
P=[250,1.4,2.4,-0.25,3,-0.1,-1.5,0,0,-30.5,1.15,1,0,0,-0.128,0,0,0,1.43];
L=[0.62,1,1,1,1,1,1,1];
n = length(Fy_front);
frontSA = zeros(1,n);    
rearSA = zeros(1,n);

for i = 1:n
    frontSA(i) = findSlipold(wheelLoads{1,1}(i),frontLoad(i),Fy_front(i),0,0,P,L)*pi/180;
    rearSA(i) = findSlipold(wheelLoads{2,1}(i),rearLoad(i),Fy_rear(i),0,0,P,L)*pi/180;
end
%% ADJUST LATERAL GRIP BASED ON DRIVE/BRAKE (TRACTION CIRCLE EFFECT)
n = length(a_x);
Fx_rear = zeros(1,n);
Fx_front = zeros(1,n);
parasiticDrag = a_y*m_tot.*sin(rearSA) + 0.5*a_y*m_tot.*sin(frontSA-rearSA) + 0.02*W_tot;
for i = 1:n
    if a_x(i) < 0
        Fx_rear(i) = (a_x(i)*m_tot + parasiticDrag(i) + DF(V))*0.3;
        Fx_front(i) = (a_x(i)*m_tot + parasiticDrag(i) + DF(V))*0.7;
    else
        Fx_rear(i) = a_x(i)*m_tot + parasiticDrag(i) + DF(V);
    end
end
Fy_front = sqrt((Fy_front).^2 + (Fx_front).^2);     %new adjusted required lateral grips
Fy_rear = sqrt((Fy_rear).^2 + (Fx_rear).^2);
%% ROLLING RESISTANCE UNDERSTEER MOMENT
Mu = 0.02*(leftLoad - rightLoad).*TF; %understeer moment from drag force (~2% of normal load)
deltaF_rollingResistance = Mu./TL;
%% UPDATE ESTIMATE OF SAs
for i = 1:n
    frontSA(i) = findSlipold(wheelLoads{1,1}(i),frontLoad(i),Fy_front(i),0,0,P,L)*pi/180;
    rearSA(i) = findSlipold(wheelLoads{2,1}(i),rearLoad(i),Fy_rear(i),0,0,P,L)*pi/180;
end
%% MZs/SELF ALIGNING TORQUE (UNDERSTEER MOMENT)
PM=[250,2.5,0.16,0.12,0,-0.065,-0.8,0,0,4.8,1.8,0,0,0,0.2,-0.01,0,0.4,0,-0.045,250];
LMZ=[0.62,1,1,1,1,1,1,1];
IA = 0*pi/180;
MZ_fo = zeros(1,n);
MZ_fi = zeros(1,n);
MZ_ro = zeros(1,n);
MZ_ri = zeros(1,n);
for i = 1:n
MZ_fo(i) = pacejkaMZ(PM,LMZ,wheelLoads{1,1}(i),IA,frontSA(i))*12;
MZ_fi(i) = pacejkaMZ(PM,LMZ,wheelLoads{1,2}(i),IA,frontSA(i))*12;
MZ_ro(i) = pacejkaMZ(PM,LMZ,wheelLoads{2,1}(i),IA,rearSA(i))*12;
MZ_ri(i) = pacejkaMZ(PM,LMZ,wheelLoads{2,2}(i),IA,rearSA(i))*12;
end

Mu = MZ_fi+MZ_fo+MZ_ro+MZ_ri; %estimate each corner using SA, Fz, and camber angle
deltaF_selfAllign = Mu./TL;
%% INDUCED TIRE DRAG (UNDERSTEER MOMENT)
Mu = (wheelLoads{1,1}-wheelLoads{1,2}).*a_y/32.2.*sin(frontSA - rearSA).*TF;
deltaF_inducedDrag = Mu./TF; 

%if alphaR > alphaF, contributes to additional oversteer (and vice versa)
%% COMPUTE REQUIRED LATERAL GRIPS
Fy_front = Fy_front + deltaF_rollingResistance + deltaF_selfAllign + deltaF_inducedDrag;
Fy_rear = Fy_rear - deltaF_rollingResistance - deltaF_selfAllign - deltaF_inducedDrag;

%% PLOT SAs USING TIRE MODEL
%first correct SAs based on understeer moments
for i = 1:n
    frontSA(i) = findSlipold(wheelLoads{1,1}(i),frontLoad(i),Fy_front(i),-1.3,-0.45,P,L);
    rearSA(i) = findSlipold(wheelLoads{2,1}(i),rearLoad(i),Fy_rear(i),0.284,-0.356,P,L);
end

figure;
plot(a_y/32.2, frontSA)
hold on
grid on
plot(a_y/32.2, rearSA)
ylabel("SA (deg)")
xlabel("Cornering g-force")
legend("Front","Rear");
hold off

figure;
plot(a_y/32.2, Fy_front)
hold on
grid on
plot(a_y/32.2, Fy_rear)
ylabel("Required Lateral Grip")
xlabel("Cornering g-force")
legend("Front","Rear");
hold off

figure;
plot(a_y/32.2, wheelLoads{1,1}-wheelLoads{1,2})
hold on
grid on
plot(a_y/32.2, wheelLoads{2,1}-wheelLoads{2,2})
ylabel("Load Transfer")
xlabel("Cornering g-force")
legend("Front","Rear");
hold off

figure;
plot(a_y/32.2, wheelLoads{1,2})
hold on
grid on
plot(a_y/32.2, wheelLoads{2,2})
ylabel("Inside Load")
xlabel("Cornering g-force")
legend("Front","Rear");
hold off

%% Steering Forces
d = 0.579; %scrub radius (in)
KPI = 7.6*pi/180; %KPI angle (rad)
theta_steer = 14*pi/180; %steering angle (rad)
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

L_arm = 3.15; %steering arm length (in)
F_rack = M_tot/(L_arm*cos(theta_steer));

r_pinion = 1.25/2; %pinion gear radius (in)
T_column = F_rack*r_pinion; %steering column torque (lbf-in)

D_wheel = 8.5; %steering wheel diameter (in)
F_wheel = -T_column/D_wheel; %steering wheel force in each hand (lbf)

%%Repeat for MF12 trail
d = 0.539; %scrub radius (in)
KPI = 8*pi/180; %KPI angle (rad)
theta_steer = 14*pi/180; %steering angle (rad)
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

L_arm = 3.15; %steering arm length (in)
F_rack = M_tot/(L_arm*cos(theta_steer));

r_pinion = 1.25/2; %pinion gear radius (in)
T_column1 = F_rack*r_pinion; %steering column torque (lbf-in)

D_wheel = 8.5; %steering wheel diameter (in)
F_wheel1 = -T_column1/D_wheel; %steering wheel force in each hand (lbf)

%%Repeat for MF11 trail
d = 0.77; %scrub radius (in)
KPI = 5.34*pi/180; %KPI angle (rad)
theta_steer = 14*pi/180; %steering angle (rad)
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

L_arm = 3.15; %steering arm length (in)
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
title("Steering Force at 14 deg Steering Angle, 8.5 in Wheel")

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
title("Steering Column Torque at 14 deg Steering Angle")