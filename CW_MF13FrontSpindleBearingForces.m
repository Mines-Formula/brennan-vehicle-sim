clear all

%Input forces from contact patch
Fx = 0; %lbf
Fy = 414; %lbf
Fz = 357; %lbf
Mz = 350; %lbf*in

%Input dimensions (in)
Td = 1.125; %depth of the triangle feature
Bd = 0.1875; %Thickness of brake rotor
r = 0.1; %radius of backside filet (the one that requires the spacer)
Bt = 15/25.4; %thickness of the bearing
Bs = 1.3945; %distance between bearing centers
Wd = 8; %Radius of wheel+tire
Cd = 1.0980; %distance(ydir) from triangle face to center of contact patch
Rs = [32,35,40,45]/25.4;


F1z = (Wd*Fy+(Bs+Bt/2+r+Bd+Td-Cd)*Fz)./Bs;
F2z = F1z-Fz;

F1x = (Mz+(Bs+Bt/2+r+Bd+Td-Cd)*Fx)./Bs;
F2x = F1x-Fx;

F1 = sqrt(F1z.^2+F1x.^2);
F2 = sqrt(F2z.^2+F2x.^2);




M1 = (Td+Bd-Cd)*Fz + 8*Fy;
sigmab1 = (M1*Rs/2);

FOS = 4000/F1

