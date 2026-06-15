%% Rear
R = linspace(0.2345,0.2355,1000); %define motion ratios to be evaluated
%note: motion ratios are defined as deg body roll / deg ARB twist
max_roll = 1.65*pi/180; %max expected body roll
k_req = 300*1000*180/pi; %N*mm/rad, target effective stiffness
k_ubar = k_req*R.^2; %ARB torsional stiffness
L = 24.2*25.4; %ubar length in mm
G = 80000; %shear modulus MPa
% k = J*G/L, J = k*L/G
J = k_ubar*L/G; %polar area MOI in mm^4
T = max_roll./R.*J*G/L;
Ss = 435*0.577; %shear strength MPa
FOS = 1.5; %desired FOS

%initialize arrays
A_mins = zeros(length(J),1);
r_outers = zeros(length(J),1);
r_inners = zeros(length(J),1);
FOSs = zeros(length(J),1);

for i = 1:length(J)
    r_o = [0];
    r_i = [0];
    A = [0];
    r_o(1) = (2/pi*J(i))^0.25; %begin by calculating required outer radius with inner radius = 0
    A(1) = pi*r_o(1)^2;
    j = 1;
    while T(i)*r_o(j)/J(i) < Ss/FOS
        j = j + 1;
        r_o(j) = r_o(j-1) + 0.01;
        r_i(j) = (-2*J(i)/pi+(r_o(j))^4)^(0.25);
        A(j) = pi*(r_o(j)^2-r_i(j)^2);
    end
    A_mins(i) = min(A);
    r_outers(i) = r_o(find(A == min(A)));
    r_inners(i) = r_i(find(A == min(A)));
    FOSs(i) = Ss/(T(i)*r_o(j)/J(i));
end



figure;
plot(R,A_mins);
title("Rear U-bar - 550 Nm/deg, FOS = 2")
xlabel("U-bar Motion Ratio in Roll [deg body roll/deg ubar twist]")
ylabel("Minimum U-bar X-section Area [mm^2]")
grid on
%{
xl = xline(R(i), 'r--', 'FOS not attainable', ...
    'LabelHorizontalAlignment', 'left', ...
    'FontSize', 18, ...          
    'FontWeight', 'bold');
%}
%% Front
R = linspace(0.19,0.21,10); %define motion ratios to be evaluated
%note: motion ratios are defined as deg body roll / deg ARB twist
max_roll = 1*pi/180; 
k_req = 175*1000*180/pi; %N*mm/rad
k_ubar = k_req*R.^2;
L = 22.8*25.4; %ubar length in mm
G = 80000; %shear modulus MPa
% k = J*G/L, J = k*L/G
J = k_ubar*L/G; %polar area MOI in mm^4
T = max_roll./R.*J*G/L;
Ss = 435*0.577; %shear strength MPa
FOS = 2;
A_mins = zeros(length(J),1);
r_outers = zeros(length(J),1);
r_inners = zeros(length(J),1);
FOSs = zeros(length(J),1);
for i = 1:length(J)
    r_o = [0];
    r_i = [0];
    A = [0];
    r_o(1) = (2/pi*J(i))^0.25;
    A(1) = pi*r_o(1)^2;
    j = 1;
    while T(i)*r_o(j)/J(i) < Ss/FOS
        j = j + 1;
        r_o(j) = r_o(j-1) + 0.01;
        r_i(j) = (-2*J(i)/pi+(r_o(j))^4)^(0.25);
        A(j) = pi*(r_o(j)^2-r_i(j)^2);
    end
    A_mins(i) = min(A);
    r_outers(i) = r_o(find(A == min(A)));
    r_inners(i) = r_i(find(A == min(A)));
    FOSs(i) = Ss/(T(i)*r_o(j)/J(i));
end

figure;
plot(R,A_mins);
title("Front")
xlabel("U-bar Motion Ratio in Roll [deg body roll/deg ubar twist]")
ylabel("Minimum U-bar X-section Area [mm^2]")
grid on


