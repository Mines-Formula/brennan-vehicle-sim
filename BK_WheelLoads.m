%all units in lbs, in unless specified

weightDistF = 0.5;
DFDistF = 0.55;
W_tot = 610;
m_uf = 37.5/32.2;  %unsprung front mass
m_ur = 40.5/32.2;  %unsprung rear mass
m_s = W_tot/32.2 - m_uf - m_ur;   %sprung mass
TL = 60.5;
TF = 48;
TR = 48;
r_tires = 8.1;
sprung_z = 11.3;
unsprung_z = r_tires;

a_y = 2*32.2; %lateral accel in ft/s^2
a_x = 0*32.2;   %long accel in ft/s^2
r_corner = 10;   %radius of corner in m
V = sqrt(r_corner.*a_y/32.2*9.81); %velocity in m/s

%{
V = linspace(5,35, 100);
a_y = V.^2./r/9.81*32.2 %lateral accel in ft/s^2
%}

LF = @(V) 50;   %downforce
DF = @(V) 15;   %drag force
crossWind = @() 0; %implement later?
drag_z = 20;
crosswind_z = 40;
crosswind_x = 10; %measured from center of rear wheels

rc_zf = 2.515; %roll center heights
rc_zr = 2.896;

kRoll_ubar = 950; %ARB stiffness in N*m/deg (950)
kRoll_tbar = 550; 

kWheel_f = 454; %wheel rate lbf/in
kWheel_r = 372; 

kTire = 720; %tire stiffness in lbf/in

kRoll_f_W = kWheel_f.*TF.^2.*tan(pi/180)/2*0.113;  %roll gradient from coilovers in N*m/deg
kRoll_r_W = kWheel_r.*TR.^2.*tan(pi/180)/2*0.113;

kRoll_f = kRoll_f_W + kRoll_ubar;  %total roll gradients, Nm/deg
kRoll_r = kRoll_r_W + kRoll_tbar;

%all deltaWs represent weight transfered from right to left
%calculate static loads, with DF, drag
W_static_rR = (W_tot*(1-weightDistF)+DF(V)*drag_z/TL+LF(V)*(1-DFDistF))/2;
W_static_rL = W_static_rR;
W_static_fR = (W_tot*(weightDistF)-DF(V)*drag_z/TL+LF(V)*(DFDistF))/2;
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
deltaW_jacking = @(SA) 7;   %function of steering angle
SA = 20; %steering angle in deg
deltaW_f_jacking = -deltaW_jacking(SA);
deltaW_r_jacking = deltaW_jacking(SA);

%Calculate individual wheel loads [fL, fR; rL, rR]

wheelLoads = cell(2,2);
wheelLoads{1,1} = W_static_fL + deltaW_scf + deltaW_sff + deltaW_uf + deltaW_f_jacking;
wheelLoads{1,2} = W_static_fL - deltaW_scf - deltaW_sff - deltaW_uf - deltaW_f_jacking;
wheelLoads{2,1} = W_static_rL + deltaW_scr + deltaW_sfr + deltaW_ur + deltaW_r_jacking;
wheelLoads{2,2} = W_static_rL - deltaW_scr - deltaW_sfr - deltaW_ur - deltaW_r_jacking;
wheelLoads

deltaW_rear = wheelLoads{2,1} - wheelLoads{2,2};
deltaW_front = wheelLoads{1,1} - wheelLoads{1,2};

%{
plot(kRoll_ubar, deltaW_rear)
hold on
plot(kRoll_ubar, deltaW_front)
hold off
%}



