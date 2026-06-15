%All units in lbf, in
%Postive x forward, positive z up, positive y left
%Forces origin at center of contact patch
%Hardpoint origin from optimum export

clear
clc
close all

%% Front
%INPUT FRONT LOADCASE HERE (applied at center of contact patch)
Fx = -164; 
Fy = -332;
Fz = 324;
Mx = 0;     
My = 0;     
Mz = 276;

%Input hardpoints
CHAS_LowFor = [35.7,8.9,4.6];
CHAS_LowAft = [25,8.9,4.6];
CHAS_UppFor = [35.5,10.2,10];
CHAS_UppAft = [24,10.2,10];
UPRI_LowPnt = [30.245,22.825,4.46];
UPRI_UppPnt = [29.755,21.875,11.575];
CHAS_TiePnt = [33.5,9.75,6.204];
UPRI_TiePnt = [33.1,23,6.65];
NSMA_PPAttPnt_L = [29.755,21,10.936];
ROCK_RodPnt_L = [29.755,10.4,3.2];

%define geometry vectors
UF = UPRI_UppPnt - CHAS_UppFor;
UA = UPRI_UppPnt - CHAS_UppAft;
LF = UPRI_LowPnt - CHAS_LowFor;
LA = UPRI_LowPnt - CHAS_LowAft;
PR = NSMA_PPAttPnt_L - ROCK_RodPnt_L;
TR = UPRI_TiePnt - CHAS_TiePnt;

%normalize geometry vector
UF1 = UF/norm(UF);
UA1 = UA/norm(UA);
LF1 = LF/norm(LF);
LA1 = LA/norm(LA);
PR1 = PR/norm(PR);
TR1 = TR/norm(TR);

% Position vectors (from origin to upright points)
OFFSET = [30,24,0];
r_UF = UPRI_UppPnt-OFFSET;
r_UA = UPRI_UppPnt-OFFSET;
r_LF = UPRI_LowPnt-OFFSET;
r_LA = UPRI_LowPnt-OFFSET;
r_PR = NSMA_PPAttPnt_L-OFFSET;
r_TR = UPRI_TiePnt-OFFSET;

% Moment contributions (r × F_unit)
m_UF = cross(r_UF, UF1);
m_UA = cross(r_UA, UA1);
m_LF = cross(r_LF, LF1);
m_LA = cross(r_LA, LA1);
m_PR = cross(r_PR, PR1);
m_TR = cross(r_TR, TR1);

%define unit vectors
i = [1;0;0];
j = [0;1;0];
k = [0;0;1];

A = [dot(UF1,i), dot(UA1,i), dot(LF1,i), dot(LA1,i), dot(PR1,i), dot(TR1,i);
     dot(UF1,j), dot(UA1,j), dot(LF1,j), dot(LA1,j), dot(PR1,j), dot(TR1,j);
     dot(UF1,k), dot(UA1,k), dot(LF1,k), dot(LA1,k), dot(PR1,k), dot(TR1,k);
     m_UF(1),   m_UA(1),   m_LF(1),   m_LA(1),   m_PR(1),   m_TR(1);
     m_UF(2),   m_UA(2),   m_LF(2),   m_LA(2),   m_PR(2),   m_TR(2);
     m_UF(3),   m_UA(3),   m_LF(3),   m_LA(3),   m_PR(3),   m_TR(3)];

b = [Fx; Fy; Fz; Mx; My; Mz];

xF = A\b; %front linkage forces [UF, UA, LF, LA, PR, TR]

%% Rear
%INPUT REAR LOADCASE HERE (applied at center of contact patch)
Fx = 0; 
Fy = 0;
Fz = 640;
Mx = 0;
My = 0;
Mz = 0;
if Fx > 0
    My = Fx*7.875; %My from driveshaft torque in accel cases
end


%Input hardpoints
CHAS_LowFor = [-21.255 11.429 5.149]; 
CHAS_LowAft = [-33.700 10.600 4.325];
CHAS_UppFor = [-21.700 11.850 9.725];
CHAS_UppAft = [-33.900 11.850 10.125];
UPRI_LowPnt = [-29.200 21.800 4.200];
UPRI_UppPnt = [-29.500 21.800 10.725];
CHAS_TiePnt = [-34.100 11.200 6.545];
UPRI_TiePnt = [-32.600 21.800 6.625];
NSMA_PPAttPnt_L = [-28.954 20.800 4.750];
ROCK_RodPnt_L = [-27.032 13.000 12.280];

%define geometry vectors
UF = UPRI_UppPnt - CHAS_UppFor;
UA = UPRI_UppPnt - CHAS_UppAft;
LF = UPRI_LowPnt - CHAS_LowFor;
LA = UPRI_LowPnt - CHAS_LowAft;
PR = NSMA_PPAttPnt_L - ROCK_RodPnt_L;
TR = UPRI_TiePnt - CHAS_TiePnt;

%normalize geometry vector
UF1 = UF/norm(UF);
UA1 = UA/norm(UA);
LF1 = LF/norm(LF);
LA1 = LA/norm(LA);
PR1 = PR/norm(PR);
TR1 = TR/norm(TR);

% Position vectors
OFFSET = [-30,24,0];
r_UF = UPRI_UppPnt-OFFSET;
r_UA = UPRI_UppPnt-OFFSET;
r_LF = UPRI_LowPnt-OFFSET;
r_LA = UPRI_LowPnt-OFFSET;
r_PR = NSMA_PPAttPnt_L-OFFSET;
r_TR = UPRI_TiePnt-OFFSET;

% Moment contributions
m_UF = cross(r_UF, UF1);
m_UA = cross(r_UA, UA1);
m_LF = cross(r_LF, LF1);
m_LA = cross(r_LA, LA1);
m_PR = cross(r_PR, PR1);
m_TR = cross(r_TR, TR1);

%unit vectors
i = [1;0;0];
j = [0;1;0];
k = [0;0;1];

A = [dot(UF1,i), dot(UA1,i), dot(LF1,i), dot(LA1,i), dot(PR1,i), dot(TR1,i);
     dot(UF1,j), dot(UA1,j), dot(LF1,j), dot(LA1,j), dot(PR1,j), dot(TR1,j);
     dot(UF1,k), dot(UA1,k), dot(LF1,k), dot(LA1,k), dot(PR1,k), dot(TR1,k);
     m_UF(1),   m_UA(1),   m_LF(1),   m_LA(1),   m_PR(1),   m_TR(1);
     m_UF(2),   m_UA(2),   m_LF(2),   m_LA(2),   m_PR(2),   m_TR(2);
     m_UF(3),   m_UA(3),   m_LF(3),   m_LA(3),   m_PR(3),   m_TR(3)];

b = [Fx; Fy; Fz; Mx; My; Mz];

xR = A\b %rear linkage forces [UF, UA, LF, LA, PR, TR]
