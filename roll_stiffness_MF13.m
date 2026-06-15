%vehicle setup
rc_z = 2.45; %roll center z (in)
sprung_W = 538; %sprung weight (lb)
sprung_z = 12.35; %sprung COM z (in)
unsprung_W = 72; %unsprung weight (lb)
unsprung_z = 8; %unsprung COM
W_tot = sprung_W + unsprung_W; %(lb)
COM_z = (unsprung_z*unsprung_W+sprung_z*sprung_W)/W_tot; %(in)


k_ubar = 0; %Nm/deg
k_tbar = 317.6; %Nm/deg
tire_rate = 650; %lbf/in


WR_front = 308; %wheel rate (lbf/in)
WR_rear = 273; %rear wheel rate (lbf/in)
TW = 48; %track width (in)
TL = 60.5; %track length (in)

%% Roll Gradient

%roll rates without ARBs (Nm/deg)
k_front = tan(pi/180)*TW^2*WR_front*0.113/2; 
k_rear = tan(pi/180)*TW^2*WR_rear*0.113/2;

%roll rate from tires (Nm/deg)
k_tires = tan(pi/180)*TW^2*tire_rate*0.113*2;

%kinematic body roll in 1g corner
Crk = (sprung_z-rc_z)*sprung_W*0.113; %kinematic roll couple (Nm)
k_tot = k_front + k_rear + k_ubar + k_tbar; %(Nm/deg)
kinematic_roll = Crk/k_tot; %(deg/g)

%roll from tires
Cr = W_tot*COM_z*0.113; %total roll couple (Nm)
tire_roll = Cr/k_tires; %(deg/g)

%total roll gradient
RG = tire_roll + kinematic_roll %(deg/g)

%% Pitch Gradient (dive)
antidive = 0; %front antidive
antilift = 0.19; %rear antilift

%longitidunal load transfer in 1g braking
deltaW = W_tot*COM_z/TL;

%front axle kinematic compression
deltaZ_front_K = deltaW/(2*WR_front)*(1-antidive);

%rear axle kinematic lift
deltaZ_rear_K = deltaW/(2*WR_rear)*(1-antilift); 

%tire compliance
deltaZ_tire = deltaW/(2*k_tires);

%total pitch dive (deg/g)
PG_dive = asind((deltaZ_tire+deltaZ_rear_K+deltaZ_front_K)/TL)

%% Pitch Gradient (squat)
antisquat = 0.066; %rear antisquat
antilift = 0; %front antilift

%longitidunal load transfer in 1g accel
deltaW = W_tot*COM_z/TL;

%front axle kinematic lift
deltaZ_front_K = deltaW/(2*WR_front)*(1-antilift);

%rear axle kinematic squat
deltaZ_rear_K = deltaW/(2*WR_rear)*(1-antisquat); 

%tire compliance
deltaZ_tire = deltaW/(2*k_tires);

%total pitch dive (deg/g)
PG_squat = asind((deltaZ_tire+deltaZ_rear_K+deltaZ_front_K)/TL)
