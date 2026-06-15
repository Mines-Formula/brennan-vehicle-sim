%vehicle setup
rc_z = 2.5; %roll center z (in)
sprung_W = 527; %sprung weight (lb)
sprung_z = 13.5; %sprung COM z (in)
unsprung_W = 74; %unsprung weight (lb)
unsprung_z = 8; %unsprung COM
W_tot = sprung_W + unsprung_W; %(lb)
COM_z = (unsprung_z*unsprung_W+sprung_z*sprung_W)/W_tot; %(in)

k_ubar = 0; %Nm/deg
k_tbar = 550; %Nm/deg
tire_rate = 680; %lbf/in

WR_front = 455; %wheel rate (lbf/in)
WR_rear = 320; %rear wheel rate (lbf/in)
TW = 48; %track width (in)

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



