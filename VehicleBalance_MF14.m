clear all;
clc;
close all;

addpath(genpath("Tire Modeling"));

overrideFile = getenv("VEHICLE_BALANCE_OVERRIDE_FILE");
overrideSpec = getenv("VEHICLE_BALANCE_OVERRIDES");
if ~isempty(overrideFile) && exist(overrideFile, "file")
    overrideParams = jsondecode(fileread(overrideFile));
    overrideSource = "file: " + string(overrideFile);
elseif ~isempty(overrideSpec)
    overrideParams = jsondecode(overrideSpec);
    overrideSource = "VEHICLE_BALANCE_OVERRIDES";
else
    overrideParams = struct();
    overrideSource = "";
end

printOverrideReport(overrideParams, overrideSource);

scriptName = mfilename;
outputDirOverride = getenv("VEHICLE_BALANCE_OUTPUT_DIR");
if isfield(overrideParams, "outputDir")
    outputDir = overrideParams.outputDir;
elseif isempty(outputDirOverride)
    outputDir = [scriptName, '_outputs'];
else
    outputDir = outputDirOverride;
end
if ~exist(outputDir, 'dir')
    mkdir(outputDir);
end

% Temporary output mode: generate only the corner-radius balance map.
generateOnlyCornerRadiusBalanceMap = true;

%% Background
% This script is a tool for estimating the steady-state cornering balance
% for a given setup. The primary tuning items of interest are weight
% distribution, aero balance, spring rates, roll centers. It also accounts
% for static toe/camber and ackerman geometry but these are secondary tuning
% paremeters.

% It solves for slip angles using a pure cornering tire model
% Add the end there are also steering force calculations

%% VEHICLE SETUP
% all units in lbs, in (unless specified)

% FIXME: This setup currently matches MF13 values. Confirm/reapply MF14
% assumptions before using this script for MF14 conclusions.

%% Mass and Weight Distribution
W_tot = 580; % weight of car and driver
weightDistF = 0.48; % percent of weight on front axle
weightDistL = 0.50; % percent of weight on left

W_tot = getOverrideValue(overrideParams, "W_tot", W_tot);
weightDistF = getOverrideValue(overrideParams, "weightDistF", weightDistF);
weightDistL = getOverrideValue(overrideParams, "weightDistL", weightDistL);

m_tot = W_tot / 32.2;
m_uf = 38.8 / 32.2;  % unsprung front mass
m_ur = 38.2 / 32.2;  % unsprung rear mass
m_uf = getOverrideValue(overrideParams, "m_uf", m_uf);
m_ur = getOverrideValue(overrideParams, "m_ur", m_ur);
m_s = W_tot / 32.2 - m_uf - m_ur;   % sprung mass

%% Vehicle Geometry
wheelbase = 60.5; % wheelbase
TF = 48; % front track
TR = 48;
r_l = 7.875; % tire loaded radius
sprung_z = 12.35; % sprung mass CG height
wheelbase = getOverrideValue(overrideParams, "wheelbase", wheelbase);
TF = getOverrideValue(overrideParams, "TF", TF);
TR = getOverrideValue(overrideParams, "TR", TR);
r_l = getOverrideValue(overrideParams, "r_l", r_l);
sprung_z = getOverrideValue(overrideParams, "sprung_z", sprung_z);
unsprung_z = r_l; % unspring mass approximatly at tire center
unsprung_z = getOverrideValue(overrideParams, "unsprung_z", unsprung_z);
CG_z = (sprung_z * m_s + unsprung_z * (m_uf + m_ur)) / m_tot; % total CG height

%% Alignment and Steering Geometry
toeF = 0; % static front toe (deg) (wheel plane to centerline)
toeR = 0;
camberF = -1.25; % front static camber (deg)
camberR = -1.25;
castor = 3.99; % (deg) used to calculate dynamic camber
KPI = 7.61; % (deg) used to calclate dynamic camber
toeF = getOverrideValue(overrideParams, "toeF", toeF);
toeR = getOverrideValue(overrideParams, "toeR", toeR);
camberF = getOverrideValue(overrideParams, "camberF", camberF);
camberR = getOverrideValue(overrideParams, "camberR", camberR);
castor = getOverrideValue(overrideParams, "castor", castor);
KPI = getOverrideValue(overrideParams, "KPI", KPI);

%% Maneuver Definition
a_x = zeros(1, 1000); % to be used once combined tire model is built
a_y = linspace(1, 2.25, 1000) * 32.2; % array of lateral accelerations to be evaluated
assert(numel(a_x) == numel(a_y), "a_x and a_y sweeps must be the same length");
n = numel(a_y);
r_corner = 30;   % radius of corner in m, measured from vehicle centerline
r_corner = getOverrideValue(overrideParams, "r_corner", r_corner);

% Derived steering geometry for the maneuver.
delta1 = atan(wheelbase ./ (r_corner / 0.0254 + TF / 2)) * 180 / pi; % outside front tire toe angle
delta2 = -delta1 * (1 + 0.002079275 * delta1) + 2 * toeF; % inside front tire toe angle
delta2Ackerman = -atan(wheelbase ./ (r_corner / 0.0254 - TF / 2)) * 180 / pi; % inside toe angle for 100% ackerman
toeEff = (delta2 - delta2Ackerman) / 2; % effective toe on front axle, accounting for ackerman

%% Aero
V = sqrt(r_corner .* a_y / 32.2 * 9.81); % velocity in m/s
CL = 3.71;
CD = 1.71;
CL = getOverrideValue(overrideParams, "CL", CL);
CD = getOverrideValue(overrideParams, "CD", CD);
CLCD = CL / CD;
DFDistFMin = 0.41;
DFDistFMax = 0.43;
% DFDistF = (DFDistFMax - DFDistFMin) / 2 + DFDistFMin;  % accounts for moment created by drag force
DFDistF = 0.43;
CLCD = getOverrideValue(overrideParams, "CLCD", CLCD);
DFDistF = getOverrideValue(overrideParams, "DFDistF", DFDistF);
LF = @(V) 1 / 2 * 1.225 * V.^2 * CL * 1.08 * 0.224809;   % downforce, lbf
DF = @(V) LF(V) / CLCD;   % drag force

%% Suspension
rc_zf = 2.329; % roll center height front
rc_zr = 2.644; % roll center height rear

kRoll_f_arb = 0; % front ARB stiffness in N*m/deg
kRoll_r_arb = 100;   % baseline rear ARB stiffness in N*m/deg
rc_zf = getOverrideValue(overrideParams, "rc_zf", rc_zf);
rc_zr = getOverrideValue(overrideParams, "rc_zr", rc_zr);
kRoll_f_arb = getOverrideValue(overrideParams, "kRoll_f_arb", kRoll_f_arb);
kRoll_r_arb = getOverrideValue(overrideParams, "kRoll_r_arb", kRoll_r_arb);

%% Parameter Sweep Mode
runParameterSweep = true;
sweepParameterNames = {'kRoll_r_arb', 'DFDistF', 'r_corner'};
sweepParameterLabels = {'Rear ARB stiffness, kRoll\_r\_arb [N*m/deg]', 'Front downforce distribution', 'Corner radius [m]'};
sweepParameterValueSets = {linspace(0, 600, 13), linspace(0.40, 0.55, 10), [5, 7.5, 10, 12.5, 15, 20, 30, 50]}; % one vector per parameter
sweepTargetG = 1.5;
sweepSampleCount = 200;
slipCapDeg = 9.999;
runParameterSweep = getOverrideValue(overrideParams, "runParameterSweep", runParameterSweep);
sweepTargetG = getOverrideValue(overrideParams, "sweepTargetG", sweepTargetG);
sweepSampleCount = getOverrideValue(overrideParams, "sweepSampleCount", sweepSampleCount);
slipCapDeg = getOverrideValue(overrideParams, "slipCapDeg", slipCapDeg);
if generateOnlyCornerRadiusBalanceMap
    runParameterSweep = false;
end

%% Corner Radius Balance Map
runCornerRadiusBalanceMap = true;
cornerRadiusMapValues = [5, 7.5, 10, 12.5, 15, 20, 30, 50, 60, 70]; % m
cornerRadiusMapRearArbValues = 0:50:300; % N*m/deg; one balance map per value
cornerRadiusMapSampleCount = 200;
runCornerRadiusBalanceMap = getOverrideValue(overrideParams, "runCornerRadiusBalanceMap", runCornerRadiusBalanceMap);
cornerRadiusMapSampleCount = getOverrideValue(overrideParams, "cornerRadiusMapSampleCount", cornerRadiusMapSampleCount);
cornerRadiusMapRearArbValues = getOverrideValue(overrideParams, "cornerRadiusMapRearArbValues", cornerRadiusMapRearArbValues);
if generateOnlyCornerRadiusBalanceMap
    runCornerRadiusBalanceMap = true;
end

%% Aero Balance Optimization
runAeroBalanceOptimization = true;
aeroOptimizationRadii = [10, 20, 30, 50]; % representative corner radii [m]
aeroOptimizationGValues = [1.25, 1.50, 1.75, 2.00];
aeroOptimizationCoarseValues = 0.30:0.025:0.55;
aeroOptimizationFineHalfRange = 0.025;
aeroOptimizationFineStep = 0.005;
aeroOptimizationTargetSlipGap = 0.10; % deg; slight-understeer target
aeroOptimizationSaturationPenalty = 4; % objective penalty per saturated fraction
aeroOptimizationWheelLiftPenalty = 4; % objective penalty per wheel-lift fraction
runAeroBalanceOptimization = getOverrideValue(overrideParams, "runAeroBalanceOptimization", runAeroBalanceOptimization);
aeroOptimizationRadii = getOverrideValue(overrideParams, "aeroOptimizationRadii", aeroOptimizationRadii);
aeroOptimizationGValues = getOverrideValue(overrideParams, "aeroOptimizationGValues", aeroOptimizationGValues);
aeroOptimizationCoarseValues = getOverrideValue(overrideParams, "aeroOptimizationCoarseValues", aeroOptimizationCoarseValues);
aeroOptimizationFineHalfRange = getOverrideValue(overrideParams, "aeroOptimizationFineHalfRange", aeroOptimizationFineHalfRange);
aeroOptimizationFineStep = getOverrideValue(overrideParams, "aeroOptimizationFineStep", aeroOptimizationFineStep);
aeroOptimizationTargetSlipGap = getOverrideValue(overrideParams, "aeroOptimizationTargetSlipGap", aeroOptimizationTargetSlipGap);

%% Front-Rear Weight Distribution Optimization
runWeightDistributionOptimization = true;
weightDistributionCoarseValues = 0.40:0.025:0.60;
weightDistributionFineHalfRange = 0.025;
weightDistributionFineStep = 0.005;
runWeightDistributionOptimization = getOverrideValue(overrideParams, "runWeightDistributionOptimization", runWeightDistributionOptimization);
weightDistributionCoarseValues = getOverrideValue(overrideParams, "weightDistributionCoarseValues", weightDistributionCoarseValues);
weightDistributionFineHalfRange = getOverrideValue(overrideParams, "weightDistributionFineHalfRange", weightDistributionFineHalfRange);
weightDistributionFineStep = getOverrideValue(overrideParams, "weightDistributionFineStep", weightDistributionFineStep);

%% Aero Balance Map Comparison
runAeroBalanceMapComparison = true;
aeroBalanceComparisonValues = [0.41, 0.43, 0.45];
runAeroBalanceMapComparison = getOverrideValue(overrideParams, "runAeroBalanceMapComparison", runAeroBalanceMapComparison);

kWheel_f = 295; % wheel rate lbf/in %370, 307.5
kWheel_r = 273; % 327, 272.5
kWheel_f = getOverrideValue(overrideParams, "kWheel_f", kWheel_f);
kWheel_r = getOverrideValue(overrideParams, "kWheel_r", kWheel_r);

kRoll_f_W = kWheel_f .* TF.^2 * tan(pi / 180) / 2 * 0.113;  % roll gradient from coilovers in N*m/deg
kRoll_r_W = kWheel_r .* TR.^2 * tan(pi / 180) / 2 * 0.113;

kRoll_f = kRoll_f_W + kRoll_f_arb;  % total roll gradients, Nm/deg
kRoll_r = kRoll_r_W + kRoll_r_arb;

% camber change from steering
IA1 = camberF - castor * sind(delta1) + KPI * (1 - cosd(delta1)); % front outside
IA2 = camberF - castor * sind(delta2) + KPI * (1 - cosd(delta2)); % front inside

% camber change from heave/roll
rollCouple = a_y * m_s * (sprung_z - (rc_zr + rc_zf) / 2);
thetaRoll = rollCouple * 0.113 / (kRoll_f + kRoll_r); % degrees of kinematic body roll
heave = LF(V) / (kWheel_f + kWheel_r); % positive = compression
IA1 = IA1 - 1.12 * heave + 0.531 * thetaRoll;  % front outside camber
IA2 = IA2 - 1.12 * heave - 0.531 * thetaRoll;  % front inside camber
IA3 = camberR - 0.972 * heave + 0.593 * thetaRoll; % rear outside camber
IA4 = camberR - 0.972 * heave - 0.593 * thetaRoll; % rear inside camber

%% CALCULATE INDIVIDUAL WHEEL LOADS
% all deltaWs represent weight transfered from inside to outside
% first calculate static loads, with downforce
W_static_rR = (W_tot * (1 - weightDistF) + LF(V) * (1 - DFDistF)) * (1 - weightDistL);
W_static_rL = (W_tot * (1 - weightDistF) + LF(V) * (1 - DFDistF)) * (weightDistL);
W_static_fR = (W_tot * (weightDistF) + LF(V) * (DFDistF)) * (1 - weightDistL);
W_static_fL = (W_tot * (weightDistF) + LF(V) * (DFDistF)) * (weightDistL);

% calculate unsprung load transfer
deltaW_uf = a_y * m_uf * r_l / TF;
deltaW_ur = a_y * m_ur * r_l / TR;

% sprung mass load transfer through suspension links
deltaW_sff = (a_y * m_s * weightDistF) * (rc_zf ./ TF);
deltaW_sfr = (a_y * m_s * (1 - weightDistF)) * (rc_zr ./ TR);

% sprung mass through springs resisting roll couple
deltaW_scf = kRoll_f ./ (kRoll_r + kRoll_f) * rollCouple ./ TF;
deltaW_scr = kRoll_r ./ (kRoll_r + kRoll_f) * rollCouple ./ TR;

% jacking from steering geometry, negative indicates outside to inside transfer
deltaW_jacking = 7 / 20 * delta1 / 2;   % function of steering angle, measured 7 at 20 deg steer
deltaW_f_jacking = -deltaW_jacking;
deltaW_r_jacking = deltaW_jacking;

% longitudinal load transfer
deltaLong = a_x * m_tot * CG_z / wheelbase;

% Calculate individual wheel loads [fL, fR; rL, rR]

wheelLoads = cell(2, 2);
wheelLoads{1, 1} = W_static_fL + deltaW_scf + deltaW_sff + deltaW_uf + deltaW_f_jacking - deltaLong / 2;
wheelLoads{1, 2} = W_static_fR - deltaW_scf - deltaW_sff - deltaW_uf - deltaW_f_jacking - deltaLong / 2;
wheelLoads{2, 1} = W_static_rL + deltaW_scr + deltaW_sfr + deltaW_ur + deltaW_r_jacking + deltaLong / 2;
wheelLoads{2, 2} = W_static_rR - deltaW_scr - deltaW_sfr - deltaW_ur - deltaW_r_jacking + deltaLong / 2;
frontLoad = wheelLoads{1, 1} + wheelLoads{1, 2};
rearLoad = wheelLoads{2, 1} + wheelLoads{2, 2};
totalLoad = frontLoad + rearLoad;
leftLoad = wheelLoads{1, 1} + wheelLoads{2, 1};
rightLoad = totalLoad - leftLoad;

deltaW_rear = wheelLoads{2, 1} - wheelLoads{2, 2};
deltaW_front = wheelLoads{1, 1} - wheelLoads{1, 2};

%% CALCULATE BASIC LATERAL GRIPS AT EACH AXLE
Fy_tot = m_tot * a_y;  % total lateral grip required, simple F=ma
Fy_front = Fy_tot * weightDistF;  % initial estimates for lateral forces at each axle
Fy_rear = Fy_tot * (1 - weightDistF);
%% INITIAL SA ESTIMATE
P = [250, 1.4, 2.4, -0.25, 3, -0.1, -1.5, 0, 0, -30.5, 1.15, 1, 0, 0, -0.128, 0, 0, 0, 1.43]; % pacejka coeffs
L = [0.62, 1, 1, 1, 1, 1, 1, 1]; % scaling factors
frontSA = zeros(1, n);
rearSA = zeros(1, n);

for i = 1:n
    % findSlip using bisection to solve for SA to satisfy inputs
    frontSA(i) = findSlip(wheelLoads{1, 1}(i), frontLoad(i), Fy_front(i), IA1(i), IA2(i), toeEff, P, L) * pi / 180;
    rearSA(i) = findSlip(wheelLoads{2, 1}(i), rearLoad(i), Fy_rear(i), IA3(i), IA4(i), toeR, P, L) * pi / 180;
end
%% ADJUST LATERAL GRIP BASED ON DRIVE/BRAKE (TRACTION CIRCLE EFFECT)
Fx_rear = zeros(1, n);
Fx_front = zeros(1, n);
parasiticDrag = a_y * m_tot .* sin(rearSA) + 0.5 * a_y * m_tot .* sin(frontSA - rearSA) + 0.02 * W_tot; % second term is usually insignificant
for i = 1:n
    if a_x(i) < 0 % braking condition, 30/70 bias
        Fx_rear(i) = (a_x(i) * m_tot + parasiticDrag(i) + DF(V(i))) * 0.3;
        Fx_front(i) = (a_x(i) * m_tot + parasiticDrag(i) + DF(V(i))) * 0.7;
    else % aceleration or constant speed condition, RWD
        Fx_rear(i) = a_x(i) * m_tot + parasiticDrag(i) + DF(V(i));
    end
end
Fy_front = sqrt((Fy_front).^2 + (Fx_front).^2);     % new adjusted required lateral grips
Fy_rear = sqrt((Fy_rear).^2 + (Fx_rear).^2);
%% ROLLING RESISTANCE UNDERSTEER MOMENT
Mu = 0.03 * (leftLoad - rightLoad) .* TF / 2; % understeer moment from drag force (~3% of normal load)
deltaF_rollingResistance = Mu ./ wheelbase;
%% UPDATE ESTIMATE OF SAs
for i = 1:n
    frontSA(i) = findSlip(wheelLoads{1, 1}(i), frontLoad(i), Fy_front(i), IA1(i), IA2(i), toeEff, P, L) * pi / 180;
    rearSA(i) = findSlip(wheelLoads{2, 1}(i), rearLoad(i), Fy_rear(i), IA3(i), IA4(i), toeR, P, L) * pi / 180;
end
%% MZs/SELF ALIGNING TORQUE (UNDERSTEER MOMENT)
% uses MZ tire model for pure cornering
PM = [250, 2.5, 0.16, 0.12, 0, -0.065, -0.8, 0, 0, 4.8, 1.8, 0, 0, 0, 0.2, -0.01, 0, 0.4, 0, -0.045, 250];
LMZ = [0.62, 1, 1, 1, 1, 1, 1, 1];
MZ_fo = zeros(1, n);
MZ_fi = zeros(1, n);
MZ_ro = zeros(1, n);
MZ_ri = zeros(1, n);
for i = 1:n
    MZ_fo(i) = pacejkaMZ(PM, LMZ, wheelLoads{1, 1}(i), IA1(i) * pi / 180, frontSA(i)) * 12;
    MZ_fi(i) = pacejkaMZ(PM, LMZ, wheelLoads{1, 2}(i), -IA2(i) * pi / 180, frontSA(i)) * 12;
    MZ_ro(i) = pacejkaMZ(PM, LMZ, wheelLoads{2, 1}(i), IA3(i) * pi / 180, rearSA(i)) * 12;
    MZ_ri(i) = pacejkaMZ(PM, LMZ, wheelLoads{2, 2}(i), -IA4(i) * pi / 180, rearSA(i)) * 12;
end

Mu = MZ_fi + MZ_fo + MZ_ro + MZ_ri; % estimate each corner using SA, Fz, and camber angle
deltaF_selfAllign = Mu ./ wheelbase;
%% INDUCED TIRE DRAG (UNDERSTEER MOMENT)
% this section could be improved to account for moment created due to
% induced tire drag at each corner (account for static toe/ackerman
% settings)
% estimate SA at each corner
SA1 = frontSA + toeEff * pi / 180;
SA2 = frontSA - toeEff * pi / 180;
SA3 = rearSA + toeR * pi / 180;
SA4 = rearSA - toeR * pi / 180;
% understeer moment due to difference in front and rear SA
Mu =  (wheelLoads{1, 1} - wheelLoads{1, 2}) .* a_y / 32.2 .* sin(frontSA - rearSA) .* TF / 2;
% understeer moment due to difference in front inner/outer induced tire drag
% Mu = Mu - (pacejka(P,L,wheelLoads{1,1},0,SA1).*sin(SA1)-pacejka(P,L,wheelLoads{1,2},0,SA2).*sin(SA2))*TF/2;
% understeer moment due to difference in rear inner/outer induced tire drag
% Mu = Mu - (pacejka(P,L,wheelLoads{2,1},0,SA3).*sin(SA3)-pacejka(P,L,wheelLoads{2,2},0,SA4).*sin(SA4))*TR/2;
deltaF_inducedDrag = Mu ./ TF;

% if alphaR > alphaF, contributes to additional oversteer (and vice versa)
% also sensitive to toe changes (i think?)
%% COMPUTE REQUIRED LATERAL GRIPS
Fy_front = Fy_front + deltaF_rollingResistance + deltaF_selfAllign + deltaF_inducedDrag;
Fy_rear = Fy_rear - deltaF_rollingResistance - deltaF_selfAllign - deltaF_inducedDrag;

%% PLOT SAs USING TIRE MODEL
% first correct SAs based on understeer moments
for i = 1:n
    frontSA(i) = findSlip(wheelLoads{1, 1}(i), frontLoad(i), Fy_front(i), IA1(i), IA2(i), toeEff, P, L);
    rearSA(i) = findSlip(wheelLoads{2, 1}(i), rearLoad(i), Fy_rear(i), IA3(i), IA4(i), toeR, P, L);
end

if ~generateOnlyCornerRadiusBalanceMap
    figure("Name", "Front vs Rear Slip Angle", "NumberTitle", "off");
    plot(a_y / 32.2, frontSA);
    hold on;
    grid on;
    plot(a_y / 32.2, rearSA);
    ylabel("SA (deg)");
    xlabel("Cornering g-force");
    title("Comparison of Front vs Rear Slip Angle");
    legend("Front", "Rear");
    subtitle({"Depicts: axle slip angles required to meet lateral force demand.", ...
              "Optimize: target a smooth, small front-rear gap; front higher than rear trends understeer, rear higher trends oversteer."});
    hold off;
    saveas(gcf, fullfile(outputDir, 'front_vs_rear_slip_angle.png'));

    figure("Name", "Required Lateral Grip", "NumberTitle", "off");
    plot(a_y / 32.2, Fy_front);
    hold on;
    grid on;
    plot(a_y / 32.2, Fy_rear);
    ylabel("Required Lateral Grip (lbf)");
    xlabel("Cornering g-force");
    title("Required Front vs Rear Lateral Grip");
    legend("Front", "Rear");
    subtitle({"Depicts: lateral force demand assigned to each axle after balance corrections.", ...
              "Optimize: shift aero, weight, roll stiffness, camber, or tire capacity toward the axle reaching its grip limit first."});
    hold off;
    saveas(gcf, fullfile(outputDir, 'required_lateral_grip.png'));

    figure("Name", "Front vs Rear Load Transfer", "NumberTitle", "off");
    plot(a_y / 32.2, wheelLoads{1, 1} - wheelLoads{1, 2});
    hold on;
    grid on;
    plot(a_y / 32.2, wheelLoads{2, 1} - wheelLoads{2, 2});
    ylabel("Load Transfer (lbf)");
    xlabel("Cornering g-force");
    title("Front vs Rear Lateral Load Transfer");
    legend("Front", "Rear");
    subtitle({"Depicts: inside-to-outside normal load transfer at each axle.", ...
              "Optimize: reduce excessive transfer at the limiting axle; lower CG, widen track, or shift roll stiffness away from that axle."});
    hold off;
    saveas(gcf, fullfile(outputDir, 'front_vs_rear_load_transfer.png'));

    figure("Name", "Inside Wheel Loads", "NumberTitle", "off");
    plot(a_y / 32.2, wheelLoads{1, 2});
    hold on;
    grid on;
    plot(a_y / 32.2, wheelLoads{2, 2});
    ylabel("Inside Load (lbf)");
    xlabel("Cornering g-force");
    title("Inside Front vs Rear Wheel Loads");
    legend("Front", "Rear");
    subtitle({"Depicts: inside tire vertical load as lateral acceleration increases.", ...
              "Optimize: keep inside loads positive and useful; avoid unloading with excessive roll stiffness, high CG, or narrow track."});
    hold off;
    saveas(gcf, fullfile(outputDir, 'inside_wheel_loads.png'));
    %% Plot Understeer Gradient
    figure("Name", "Understeer Gradient", "NumberTitle", "off");
    theta_steer = frontSA - rearSA + (abs(delta1) + abs(delta2)) / 2;
    plot(a_y / 32.2, theta_steer);
    xlabel("Lateral Acceleration [g]");
    ylabel("Steering Angle [deg]");
    title(sprintf("Understeer Gradient for %.0f m Radius Corner", r_corner));
    subtitle({"Depicts: steering angle required as lateral acceleration rises.", ...
              "Optimize: target a smooth, predictable, slightly positive gradient; reduce slope for less understeer and avoid negative slope at the limit."});
    saveas(gcf, fullfile(outputDir, 'understeer_gradient.png'));
end

%% Parameter Sweep Results
if runParameterSweep
    sweepBaseParams = struct( ...
                             'W_tot', W_tot, ...
                             'weightDistF', weightDistF, ...
                             'weightDistL', weightDistL, ...
                             'm_uf', m_uf, ...
                             'm_ur', m_ur, ...
                             'wheelbase', wheelbase, ...
                             'TF', TF, ...
                             'TR', TR, ...
                             'r_l', r_l, ...
                             'sprung_z', sprung_z, ...
                             'toeF', toeF, ...
                             'toeR', toeR, ...
                             'camberF', camberF, ...
                             'camberR', camberR, ...
                             'castor', castor, ...
                             'KPI', KPI, ...
                             'a_x', a_x, ...
                             'a_y', a_y, ...
                             'r_corner', r_corner, ...
                             'CL', CL, ...
                             'CLCD', CLCD, ...
                             'DFDistF', DFDistF, ...
                             'rc_zf', rc_zf, ...
                             'rc_zr', rc_zr, ...
                             'kRoll_f_arb', kRoll_f_arb, ...
                             'kRoll_r_arb', kRoll_r_arb, ...
                             'kWheel_f', kWheel_f, ...
                             'kWheel_r', kWheel_r);

    assert(numel(sweepParameterNames) == numel(sweepParameterLabels), "Each sweep parameter needs one label");
    assert(numel(sweepParameterNames) == numel(sweepParameterValueSets), "Each sweep parameter needs one value vector");

    for sweepDefIdx = 1:numel(sweepParameterNames)
        sweepParameterName = sweepParameterNames{sweepDefIdx};
        sweepParameterLabel = sweepParameterLabels{sweepDefIdx};
        sweepParameterValues = sweepParameterValueSets{sweepDefIdx};

        assert(isfield(sweepBaseParams, sweepParameterName), "sweepParameterName must match a field in sweepBaseParams");

        frontLimitG = nan(size(sweepParameterValues));
        rearLimitG = nan(size(sweepParameterValues));
        slipGapAtTarget = nan(size(sweepParameterValues));

        for sweepIdx = 1:numel(sweepParameterValues)
            sweepCaseParams = sweepBaseParams;
            sweepCaseParams.(sweepParameterName) = sweepParameterValues(sweepIdx);
            sweepResult = evaluateVehicleBalance(sweepCaseParams, P, L, PM, LMZ, sweepSampleCount);

            frontLimitIdx = find(sweepResult.frontSA >= slipCapDeg, 1, 'first');
            rearLimitIdx = find(sweepResult.rearSA >= slipCapDeg, 1, 'first');
            if isempty(frontLimitIdx)
                frontLimitG(sweepIdx) = sweepResult.g(end);
            else
                frontLimitG(sweepIdx) = sweepResult.g(frontLimitIdx);
            end
            if isempty(rearLimitIdx)
                rearLimitG(sweepIdx) = sweepResult.g(end);
            else
                rearLimitG(sweepIdx) = sweepResult.g(rearLimitIdx);
            end
            slipGapAtTarget(sweepIdx) = interp1(sweepResult.g, sweepResult.frontSA - sweepResult.rearSA, sweepTargetG, 'linear', 'extrap');
        end

        figure("Name", sprintf("%s Sweep", sweepParameterName), "NumberTitle", "off");
        subplot(2, 1, 1);
        plot(sweepParameterValues, frontLimitG, "-o");
        hold on;
        plot(sweepParameterValues, rearLimitG, "-o");
        grid on;
        ylabel("Slip cap g");
        title(sprintf("%s Sweep: Axle Limit Estimate", sweepParameterName));
        legend("Front 10 deg SA", "Rear 10 deg SA", "Location", "best");
        subtitle("Higher g before the 10 deg slip cap indicates more axle margin in this model.");
        hold off;

        subplot(2, 1, 2);
        plot(sweepParameterValues, slipGapAtTarget, "-o");
        grid on;
        xlabel(sweepParameterLabel);
        ylabel(sprintf("Front - rear SA at %.1f g [deg]", sweepTargetG));
        title("Balance at Target Lateral Acceleration");
        subtitle("Positive values trend understeer; negative values trend oversteer.");
        yline(0, "--");
        sweepOutputName = [regexprep(sweepParameterName, '[^A-Za-z0-9_]', '_'), '_sweep.png'];
        saveas(gcf, fullfile(outputDir, sweepOutputName));
    end
end

%% Corner Radius Balance Map
if runCornerRadiusBalanceMap
    radiusMapBaseParams = struct( ...
                                 'W_tot', W_tot, ...
                                 'weightDistF', weightDistF, ...
                                 'weightDistL', weightDistL, ...
                                 'm_uf', m_uf, ...
                                 'm_ur', m_ur, ...
                                 'wheelbase', wheelbase, ...
                                 'TF', TF, ...
                                 'TR', TR, ...
                                 'r_l', r_l, ...
                                 'sprung_z', sprung_z, ...
                                 'toeF', toeF, ...
                                 'toeR', toeR, ...
                                 'camberF', camberF, ...
                                 'camberR', camberR, ...
                                 'castor', castor, ...
                                 'KPI', KPI, ...
                                 'a_x', a_x, ...
                                 'a_y', a_y, ...
                                 'r_corner', r_corner, ...
                                 'CL', CL, ...
                                 'CLCD', CLCD, ...
                                 'DFDistF', DFDistF, ...
                                 'rc_zf', rc_zf, ...
                                 'rc_zr', rc_zr, ...
                                 'kRoll_f_arb', kRoll_f_arb, ...
                                 'kRoll_r_arb', kRoll_r_arb, ...
                                 'kWheel_f', kWheel_f, ...
                                 'kWheel_r', kWheel_r);

    if runAeroBalanceOptimization
        aeroOptimizationBaseParams = radiusMapBaseParams;
        aeroOptimizationBaseParams.a_y = aeroOptimizationGValues * 32.2;
        aeroOptimizationBaseParams.a_x = zeros(size(aeroOptimizationGValues));
        coarseAeroMetrics = evaluateBalanceParameterCandidates( ...
            aeroOptimizationCoarseValues, "DFDistF", aeroOptimizationRadii, aeroOptimizationBaseParams, ...
            P, L, PM, LMZ, numel(aeroOptimizationGValues), slipCapDeg, ...
            aeroOptimizationTargetSlipGap, aeroOptimizationSaturationPenalty, ...
            aeroOptimizationWheelLiftPenalty);
        [~, coarseBestIdx] = min(coarseAeroMetrics.objective);
        coarseBestValue = aeroOptimizationCoarseValues(coarseBestIdx);
        fineAeroValues = unique(max(0, coarseBestValue - aeroOptimizationFineHalfRange): ...
                                aeroOptimizationFineStep: ...
                                min(1, coarseBestValue + aeroOptimizationFineHalfRange));
        fineAeroMetrics = evaluateBalanceParameterCandidates( ...
            fineAeroValues, "DFDistF", aeroOptimizationRadii, aeroOptimizationBaseParams, ...
            P, L, PM, LMZ, numel(aeroOptimizationGValues), slipCapDeg, ...
            aeroOptimizationTargetSlipGap, aeroOptimizationSaturationPenalty, ...
            aeroOptimizationWheelLiftPenalty);
        [bestAeroObjective, bestAeroIdx] = min(fineAeroMetrics.objective);
        optimalDFDistF = fineAeroValues(bestAeroIdx);

        fprintf("Aero optimization: optimal DFDistF = %.3f (objective %.3f deg-equivalent)\n", ...
                optimalDFDistF, bestAeroObjective);
        fprintf("  RMS slip-gap error = %.3f deg, saturation = %.1f%%, wheel lift = %.1f%%\n", ...
                fineAeroMetrics.rmsSlipGapError(bestAeroIdx), ...
                100 * fineAeroMetrics.saturationFraction(bestAeroIdx), ...
                100 * fineAeroMetrics.wheelLiftFraction(bestAeroIdx));

        figure("Name", "Aero Balance Optimization", "NumberTitle", "off");
        subplot(2, 1, 1);
        plot(aeroOptimizationCoarseValues, coarseAeroMetrics.objective, "o-", "DisplayName", "Coarse search");
        hold on;
        plot(fineAeroValues, fineAeroMetrics.objective, ".-", "LineWidth", 1.5, "MarkerSize", 14, "DisplayName", "Fine search");
        xline(optimalDFDistF, "--", sprintf("Optimum %.3f", optimalDFDistF));
        grid on;
        ylabel("Penalized objective [deg-equivalent]");
        title(sprintf("Aero Balance Optimization at Rear ARB = %.0f N*m/deg", kRoll_r_arb));
        legend("Location", "best");
        hold off;

        subplot(2, 1, 2);
        plot(fineAeroValues, fineAeroMetrics.rmsSlipGapError, "o-", "DisplayName", "RMS balance error [deg]");
        hold on;
        plot(fineAeroValues, 100 * fineAeroMetrics.saturationFraction, "o-", "DisplayName", "Saturated points [%]");
        plot(fineAeroValues, 100 * fineAeroMetrics.wheelLiftFraction, "o-", "DisplayName", "Wheel-lift points [%]");
        xline(optimalDFDistF, "--");
        grid on;
        xlabel("Front downforce distribution");
        ylabel("Metric value");
        legend("Location", "best");
        subtitle(sprintf("Target gap = %.2f deg across radii %s m and g points %s", ...
                         aeroOptimizationTargetSlipGap, mat2str(aeroOptimizationRadii), ...
                         mat2str(aeroOptimizationGValues)));
        hold off;
        saveas(gcf, fullfile(outputDir, 'aero_balance_optimization.png'));
    end

    if runWeightDistributionOptimization
        weightOptimizationBaseParams = radiusMapBaseParams;
        weightOptimizationBaseParams.a_y = aeroOptimizationGValues * 32.2;
        weightOptimizationBaseParams.a_x = zeros(size(aeroOptimizationGValues));
        coarseWeightMetrics = evaluateBalanceParameterCandidates( ...
            weightDistributionCoarseValues, "weightDistF", aeroOptimizationRadii, weightOptimizationBaseParams, ...
            P, L, PM, LMZ, numel(aeroOptimizationGValues), slipCapDeg, ...
            aeroOptimizationTargetSlipGap, aeroOptimizationSaturationPenalty, ...
            aeroOptimizationWheelLiftPenalty);
        [~, coarseWeightBestIdx] = min(coarseWeightMetrics.objective);
        coarseWeightBestValue = weightDistributionCoarseValues(coarseWeightBestIdx);
        fineWeightValues = unique(max(0, coarseWeightBestValue - weightDistributionFineHalfRange): ...
                                  weightDistributionFineStep: ...
                                  min(1, coarseWeightBestValue + weightDistributionFineHalfRange));
        fineWeightMetrics = evaluateBalanceParameterCandidates( ...
            fineWeightValues, "weightDistF", aeroOptimizationRadii, weightOptimizationBaseParams, ...
            P, L, PM, LMZ, numel(aeroOptimizationGValues), slipCapDeg, ...
            aeroOptimizationTargetSlipGap, aeroOptimizationSaturationPenalty, ...
            aeroOptimizationWheelLiftPenalty);
        [bestWeightObjective, bestWeightIdx] = min(fineWeightMetrics.objective);
        optimalWeightDistF = fineWeightValues(bestWeightIdx);

        fprintf("Weight distribution optimization: %.1f%% front / %.1f%% rear (objective %.3f deg-equivalent)\n", ...
                100 * optimalWeightDistF, 100 * (1 - optimalWeightDistF), bestWeightObjective);
        fprintf("  RMS slip-gap error = %.3f deg, saturation = %.1f%%, wheel lift = %.1f%%\n", ...
                fineWeightMetrics.rmsSlipGapError(bestWeightIdx), ...
                100 * fineWeightMetrics.saturationFraction(bestWeightIdx), ...
                100 * fineWeightMetrics.wheelLiftFraction(bestWeightIdx));

        figure("Name", "Front-Rear Weight Distribution Optimization", "NumberTitle", "off");
        subplot(2, 1, 1);
        plot(100 * weightDistributionCoarseValues, coarseWeightMetrics.objective, "o-", "DisplayName", "Coarse search");
        hold on;
        plot(100 * fineWeightValues, fineWeightMetrics.objective, ".-", "LineWidth", 1.5, ...
             "MarkerSize", 14, "DisplayName", "Fine search");
        xline(100 * optimalWeightDistF, "--", sprintf("Optimum %.1f%% front", 100 * optimalWeightDistF));
        xline(100 * weightDistF, ":", sprintf("Baseline %.1f%% front", 100 * weightDistF));
        grid on;
        ylabel("Penalized objective [deg-equivalent]");
        title("MF14 Front-Rear Weight Distribution Optimization");
        legend("Location", "best");
        hold off;

        subplot(2, 1, 2);
        plot(100 * fineWeightValues, fineWeightMetrics.rmsSlipGapError, "o-", "DisplayName", "RMS balance error [deg]");
        hold on;
        plot(100 * fineWeightValues, 100 * fineWeightMetrics.saturationFraction, "o-", "DisplayName", "Saturated points [%]");
        plot(100 * fineWeightValues, 100 * fineWeightMetrics.wheelLiftFraction, "o-", "DisplayName", "Wheel-lift points [%]");
        xline(100 * optimalWeightDistF, "--");
        grid on;
        xlabel("Front weight distribution [%]");
        ylabel("Metric value");
        legend("Location", "best");
        subtitle(sprintf("Target gap = %.2f deg across radii %s m and g points %s", ...
                         aeroOptimizationTargetSlipGap, mat2str(aeroOptimizationRadii), ...
                         mat2str(aeroOptimizationGValues)));
        hold off;
        saveas(gcf, fullfile(outputDir, 'weight_distribution_optimization.png'));
    end

    if runAeroBalanceMapComparison
        aeroComparisonMaps = cell(size(aeroBalanceComparisonValues));
        combinedAeroComparisonGap = [];
        for aeroComparisonIdx = 1:numel(aeroBalanceComparisonValues)
            aeroComparisonParams = radiusMapBaseParams;
            aeroComparisonParams.DFDistF = aeroBalanceComparisonValues(aeroComparisonIdx);
            aeroComparisonMaps{aeroComparisonIdx} = calculateCornerRadiusMap( ...
                aeroComparisonParams, cornerRadiusMapValues, P, L, PM, LMZ, ...
                cornerRadiusMapSampleCount, slipCapDeg);
            currentFiniteGap = aeroComparisonMaps{aeroComparisonIdx}.displayGap( ...
                isfinite(aeroComparisonMaps{aeroComparisonIdx}.displayGap));
            combinedAeroComparisonGap = [combinedAeroComparisonGap; currentFiniteGap]; %#ok<AGROW>
        end
        if isempty(combinedAeroComparisonGap)
            aeroComparisonColorLimit = 1;
        else
            aeroComparisonColorLimit = max(abs(combinedAeroComparisonGap));
        end

        aeroComparisonFigure = figure("Name", "MF14 Aero Balance Comparison", ...
                                      "NumberTitle", "off", "Position", [50, 100, 2200, 750]);
        aeroComparisonLayout = tiledlayout(aeroComparisonFigure, 1, 3, ...
                                           "TileSpacing", "compact", "Padding", "compact");
        for aeroComparisonIdx = 1:numel(aeroBalanceComparisonValues)
            aeroComparisonAxes = nexttile(aeroComparisonLayout);
            plotCornerRadiusBalancePanel(aeroComparisonAxes, aeroComparisonMaps{aeroComparisonIdx}, ...
                cornerRadiusMapValues, aeroComparisonColorLimit, ...
                sprintf("%.0f%% front aero balance", 100 * aeroBalanceComparisonValues(aeroComparisonIdx)));
            if aeroComparisonIdx == numel(aeroBalanceComparisonValues)
                aeroComparisonColorbar = colorbar(aeroComparisonAxes);
                aeroComparisonColorbar.Label.String = "Front - rear slip angle [deg]";
            end
        end
        aeroComparisonLayout.Title.String = sprintf( ...
            "MF14 Aero Balance Comparison: Rear ARB = %.0f N*m/deg", kRoll_r_arb);
        aeroComparisonLayout.Subtitle.String = ...
            "Shared scale and operating grid; right arrows indicate understeer, left arrows indicate oversteer, gray marks slip cap.";
        saveas(aeroComparisonFigure, fullfile(outputDir, 'aero_balance_comparison_41_43_45.png'));
    end

    % MF13 comparison parameters. Both vehicles use the MF14 g/radius grid
    % and the same rear-ARB value so each panel is directly comparable.
    mf13RadiusMapBaseParams = radiusMapBaseParams;
    mf13RadiusMapBaseParams.weightDistF = 0.49;
    mf13RadiusMapBaseParams.weightDistL = 0.51;
    mf13RadiusMapBaseParams.m_uf = 37.5 / 32.2;
    mf13RadiusMapBaseParams.m_ur = 40.5 / 32.2;
    mf13RadiusMapBaseParams.castor = 4;
    mf13RadiusMapBaseParams.KPI = 7.6;
    mf13RadiusMapBaseParams.CL = 3.05;
    mf13RadiusMapBaseParams.CLCD = 2;
    mf13RadiusMapBaseParams.DFDistF = 0.46;
    mf13RadiusMapBaseParams.kWheel_f = 307.5;
    mf13RadiusMapBaseParams.kWheel_r = 272.5;

    for arbMapIdx = 1:numel(cornerRadiusMapRearArbValues)
        currentRearArb = cornerRadiusMapRearArbValues(arbMapIdx);
        radiusMapBaseParams.kRoll_r_arb = currentRearArb;
        mf13RadiusMapBaseParams.kRoll_r_arb = currentRearArb;

        mf14Map = calculateCornerRadiusMap(radiusMapBaseParams, cornerRadiusMapValues, ...
            P, L, PM, LMZ, cornerRadiusMapSampleCount, slipCapDeg);
        mf13Map = calculateCornerRadiusMap(mf13RadiusMapBaseParams, cornerRadiusMapValues, ...
            P, L, PM, LMZ, cornerRadiusMapSampleCount, slipCapDeg);

        combinedFiniteGap = [mf14Map.displayGap(isfinite(mf14Map.displayGap)); ...
                             mf13Map.displayGap(isfinite(mf13Map.displayGap))];
        if isempty(combinedFiniteGap)
            sharedColorLimit = 1;
        else
            sharedColorLimit = max(abs(combinedFiniteGap));
        end

        comparisonFigure = figure("Name", "MF14 vs MF13 Corner Radius Balance", ...
                                  "NumberTitle", "off", "Position", [100, 100, 1800, 800]);
        comparisonLayout = tiledlayout(comparisonFigure, 1, 2, "TileSpacing", "compact", "Padding", "compact");
        mf14Axes = nexttile(comparisonLayout);
        plotCornerRadiusBalancePanel(mf14Axes, mf14Map, cornerRadiusMapValues, ...
                                     sharedColorLimit, "MF14 parameters");
        mf13Axes = nexttile(comparisonLayout);
        plotCornerRadiusBalancePanel(mf13Axes, mf13Map, cornerRadiusMapValues, ...
                                     sharedColorLimit, "MF13 parameters");
        comparisonColorbar = colorbar(mf13Axes);
        comparisonColorbar.Label.String = "Front - rear slip angle [deg]";
        comparisonLayout.Title.String = sprintf("MF14 vs MF13 Balance: Rear ARB = %.0f N*m/deg", currentRearArb);
        comparisonLayout.Subtitle.String = "Shared color scale and operating grid; white contours show speed [mph], gray marks the slip cap.";

        mapOutputName = sprintf('corner_radius_balance_MF14_vs_MF13_kRoll_r_arb_%g.png', currentRearArb);
        saveas(comparisonFigure, fullfile(outputDir, mapOutputName));
    end
end
%% Steering Forces
d = 0.579; % scrub radius (in)
KPI = 7.6 * pi / 180; % KPI angle (rad)
theta_steer = 10 * pi / 180; % steering angle (rad)
castor = 3.94 * pi / 180; % castor angle (rad)
trail = 0.552; % mechanical trail (in)
r = 7.875; % tire radius (in)

% moment due to vertical force
M_V = -(wheelLoads{1, 1} + wheelLoads{1, 2}) * d * sin(KPI) * sin(theta_steer) + ...
    (wheelLoads{1, 1} - wheelLoads{1, 2}) * d * sin(castor) * sin(theta_steer);

% moment due to lateral force
M_L = -(Fy_front) * trail;

% monent due to aligning torque
M_AT = -(MZ_fi + MZ_fo) * cos(sqrt(KPI^2 + castor^2));

M_tot = M_V + M_L + M_AT;

L_arm = 2.95; % steering arm length (in)
F_rack = M_tot / (L_arm * cos(theta_steer));

r_pinion = 1.25 / 2; % pinion gear radius (in)
T_column = F_rack * r_pinion; % steering column torque (lbf-in)

D_wheel = 8.5; % steering wheel diameter (in)
F_wheel = -T_column / D_wheel; % steering wheel force in each hand (lbf)

%% Repeat for MF12 trail
d = 0.539; % scrub radius (in)
KPI = 8 * pi / 180; % KPI angle (rad)
theta_steer = 10 * pi / 180; % steering angle (rad)
castor = 4 * pi / 180; % castor angle (rad)
trail = 0.752; % mechanical trail (in)
r = 7.875; % tire radius (in)

% moment due to vertical force
M_V = -(wheelLoads{1, 1} + wheelLoads{1, 2}) * d * sin(KPI) * sin(theta_steer) + ...
    (wheelLoads{1, 1} - wheelLoads{1, 2}) * d * sin(castor) * sin(theta_steer);

% moment due to lateral force
M_L = -(Fy_front) * trail;

% monent due to aligning torque
M_AT = -(MZ_fi + MZ_fo) * cos(sqrt(KPI^2 + castor^2));

M_tot = M_V + M_L + M_AT;

L_arm = 2.95; % steering arm length (in)
F_rack = M_tot / (L_arm * cos(theta_steer));

r_pinion = 1.25 / 2; % pinion gear radius (in)
T_column1 = F_rack * r_pinion; % steering column torque (lbf-in)

D_wheel = 8.5; % steering wheel diameter (in)
F_wheel1 = -T_column1 / D_wheel; % steering wheel force in each hand (lbf)

%% Repeat for MF11 trail
d = 0.77; % scrub radius (in)
KPI = 5.34 * pi / 180; % KPI angle (rad)
theta_steer = 10 * pi / 180; % steering angle (rad)
castor = 2 * pi / 180; % castor angle (rad)
trail = 0.475; % mechanical trail (in)
r = 7.875; % tire radius (in)

% moment due to vertical force
M_V = -(wheelLoads{1, 1} + wheelLoads{1, 2}) * d * sin(KPI) * sin(theta_steer) + ...
    (wheelLoads{1, 1} - wheelLoads{1, 2}) * d * sin(castor) * sin(theta_steer);

% moment due to lateral force
M_L = -(Fy_front) * trail;

% monent due to aligning torque
M_AT = -(MZ_fi + MZ_fo) * cos(sqrt(KPI^2 + castor^2));

M_tot = M_V + M_L + M_AT;

L_arm = 2.95; % steering arm length (in)
F_rack = M_tot / (L_arm * cos(theta_steer));

r_pinion = 1.25 / 2; % pinion gear radius (in)
T_column2 = F_rack * r_pinion; % steering column torque (lbf-in)

D_wheel = 8.5; % steering wheel diameter (in)
F_wheel2 = -T_column2 / D_wheel; % steering wheel force in each hand (lbf)

if ~generateOnlyCornerRadiusBalanceMap
    figure("Name", "Steering Wheel Force Comparison", "NumberTitle", "off");
    plot(a_y / 32.2, F_wheel);
    hold on;
    plot(a_y / 32.2, F_wheel1);
    plot(a_y / 32.2, F_wheel2);
    xlim([0.8, 1.9]);
    xlabel("Cornering G-force");
    ylabel("Steering Force (lbf)");
    legend("MF14", "MF12", "MF11");
    grid on;
    title("Steering Force at 10 deg Steering Angle, 8.5 in Wheel");
    subtitle({"Depicts: driver hand force from vertical load, lateral force, and aligning torque.", ...
              "Optimize: keep effort high enough for feedback but not fatiguing; tune trail, scrub radius, caster, KPI, and steering ratio."});
    saveas(gcf, fullfile(outputDir, 'steering_wheel_force_comparison.png'));

    figure("Name", "Steering Column Torque Comparison", "NumberTitle", "off");
    plot(a_y / 32.2, -T_column);
    hold on;
    plot(a_y / 32.2, -T_column1);
    plot(a_y / 32.2, -T_column2);
    xlim([0.8, 1.9]);
    xlabel("Cornering G-force");
    ylabel("Column Torque (lbf-in)");
    legend("MF14", "MF12", "MF11");
    grid on;
    title("Steering Column Torque at 10 deg Steering Angle");
    subtitle({"Depicts: steering column torque required for each front geometry case.", ...
              "Optimize: balance feedback and effort by tuning trail, scrub radius, caster, KPI, and steering ratio."});
    saveas(gcf, fullfile(outputDir, 'steering_column_torque_comparison.png'));
end

function mapResult = calculateCornerRadiusMap(baseParams, radiusValues, ...
                                              P, L, PM, LMZ, sampleCount, slipCapDeg)
    for radiusIdx = 1:numel(radiusValues)
        radiusParams = baseParams;
        radiusParams.r_corner = radiusValues(radiusIdx);
        radiusResult = evaluateVehicleBalance(radiusParams, P, L, PM, LMZ, sampleCount);
        if radiusIdx == 1
            mapResult.g = radiusResult.g;
            mapResult.slipGap = nan(numel(radiusValues), numel(mapResult.g));
            mapResult.saturated = false(numel(radiusValues), numel(mapResult.g));
        end
        mapResult.slipGap(radiusIdx, :) = radiusResult.frontSA - radiusResult.rearSA;
        mapResult.saturated(radiusIdx, :) = radiusResult.frontSA >= slipCapDeg | ...
                                                  radiusResult.rearSA >= slipCapDeg;
    end
    mapResult.displayGap = mapResult.slipGap;
    mapResult.displayGap(mapResult.saturated) = NaN;
end

function plotCornerRadiusBalancePanel(ax, mapResult, radiusValues, colorLimit, panelTitle)
    arrowThreshold = 0.05;
    balanceImage = imagesc(ax, mapResult.g, radiusValues, mapResult.displayGap);
    set(balanceImage, "AlphaData", isfinite(mapResult.displayGap));
    set(ax, "YDir", "normal");
    clim(ax, [-colorLimit, colorLimit]);
    hold(ax, "on");

    [gGrid, radiusGrid] = meshgrid(mapResult.g, radiusValues);
    speedMphMap = sqrt(radiusGrid .* gGrid * 9.81) * 2.23694;
    [speedContour, speedContourHandle] = contour(ax, mapResult.g, radiusValues, speedMphMap, ...
                                                  15:5:75, "w--");
    clabel(speedContour, speedContourHandle, "Color", "w", "FontWeight", "bold");

    if any(mapResult.saturated(:))
        scatter(ax, gGrid(mapResult.saturated), radiusGrid(mapResult.saturated), 55, ...
                "s", "filled", "MarkerFaceColor", [0.1, 0.1, 0.1], ...
                "MarkerEdgeColor", "none", "MarkerFaceAlpha", 0.35);
    end
    contour(ax, mapResult.g, radiusValues, mapResult.displayGap, ...
            [-arrowThreshold, arrowThreshold], "k:", "LineWidth", 1.2);

    arrowColIdx = unique(round(linspace(1, numel(mapResult.g), 13)));
    arrowGap = mapResult.displayGap(:, arrowColIdx);
    finiteArrowGap = arrowGap(isfinite(arrowGap));
    if ~isempty(finiteArrowGap)
        arrowScale = max(abs(finiteArrowGap));
        [arrowG, arrowRadius] = meshgrid(mapResult.g(arrowColIdx), radiusValues);
        arrowMask = isfinite(arrowGap) & abs(arrowGap) >= arrowThreshold;
        arrowGap = arrowGap(arrowMask);
        arrowG = arrowG(arrowMask);
        arrowRadius = arrowRadius(arrowMask);
        for arrowIdx = 1:numel(arrowGap)
            if arrowGap(arrowIdx) > 0
                arrowText = "\rightarrow";
            else
                arrowText = "\leftarrow";
            end
            arrowSize = 11 + 5 * min(abs(arrowGap(arrowIdx)) / arrowScale, 1);
            text(ax, arrowG(arrowIdx), arrowRadius(arrowIdx), arrowText, ...
                 "Color", "w", "FontSize", arrowSize, "FontWeight", "bold", ...
                 "HorizontalAlignment", "center", "VerticalAlignment", "middle");
        end
    end

    xlabel(ax, "Lateral acceleration [g]");
    ylabel(ax, "Corner radius [m]");
    title(ax, panelTitle);
    grid(ax, "off");
    hold(ax, "off");
end

function result = evaluateVehicleBalance(params, P, L, PM, LMZ, sampleCount)
    sampleIdx = unique(round(linspace(1, numel(params.a_y), min(sampleCount, numel(params.a_y)))));

    a_y = params.a_y(sampleIdx);
    a_x = params.a_x(sampleIdx);
    n = numel(a_y);
    m_tot = params.W_tot / 32.2;
    m_s = m_tot - params.m_uf - params.m_ur;
    unsprung_z = params.r_l;
    CG_z = (params.sprung_z * m_s + unsprung_z * (params.m_uf + params.m_ur)) / m_tot;

    delta1 = atan(params.wheelbase ./ (params.r_corner / 0.0254 + params.TF / 2)) * 180 / pi;
    delta2 = -delta1 * (1 + 0.002079275 * delta1) + 2 * params.toeF;
    delta2Ackerman = -atan(params.wheelbase ./ (params.r_corner / 0.0254 - params.TF / 2)) * 180 / pi;
    toeEff = (delta2 - delta2Ackerman) / 2;

    V = sqrt(params.r_corner .* a_y / 32.2 * 9.81);
    LF = @(V) 1 / 2 * 1.225 * V.^2 * params.CL * 1.08 * 0.224809;
    DF = @(V) LF(V) / params.CLCD;

    kRoll_f_W = params.kWheel_f .* params.TF.^2 * tan(pi / 180) / 2 * 0.113;
    kRoll_r_W = params.kWheel_r .* params.TR.^2 * tan(pi / 180) / 2 * 0.113;
    kRoll_f = kRoll_f_W + params.kRoll_f_arb;
    kRoll_r = kRoll_r_W + params.kRoll_r_arb;

    IA1 = params.camberF - params.castor * sind(delta1) + params.KPI * (1 - cosd(delta1));
    IA2 = params.camberF - params.castor * sind(delta2) + params.KPI * (1 - cosd(delta2));

    rollCouple = a_y * m_s * (params.sprung_z - (params.rc_zr + params.rc_zf) / 2);
    thetaRoll = rollCouple * 0.113 / (kRoll_f + kRoll_r);
    heave = LF(V) / (params.kWheel_f + params.kWheel_r);
    IA1 = IA1 - 1.12 * heave + 0.531 * thetaRoll;
    IA2 = IA2 - 1.12 * heave - 0.531 * thetaRoll;
    IA3 = params.camberR - 0.972 * heave + 0.593 * thetaRoll;
    IA4 = params.camberR - 0.972 * heave - 0.593 * thetaRoll;

    W_static_rR = (params.W_tot * (1 - params.weightDistF) + LF(V) * (1 - params.DFDistF)) * (1 - params.weightDistL);
    W_static_rL = (params.W_tot * (1 - params.weightDistF) + LF(V) * (1 - params.DFDistF)) * params.weightDistL;
    W_static_fR = (params.W_tot * params.weightDistF + LF(V) * params.DFDistF) * (1 - params.weightDistL);
    W_static_fL = (params.W_tot * params.weightDistF + LF(V) * params.DFDistF) * params.weightDistL;

    deltaW_uf = a_y * params.m_uf * params.r_l / params.TF;
    deltaW_ur = a_y * params.m_ur * params.r_l / params.TR;
    deltaW_sff = (a_y * m_s * params.weightDistF) * (params.rc_zf ./ params.TF);
    deltaW_sfr = (a_y * m_s * (1 - params.weightDistF)) * (params.rc_zr ./ params.TR);
    deltaW_scf = kRoll_f ./ (kRoll_r + kRoll_f) * rollCouple ./ params.TF;
    deltaW_scr = kRoll_r ./ (kRoll_r + kRoll_f) * rollCouple ./ params.TR;
    deltaW_jacking = 7 / 20 * delta1 / 2;
    deltaW_f_jacking = -deltaW_jacking;
    deltaW_r_jacking = deltaW_jacking;
    deltaLong = a_x * m_tot * CG_z / params.wheelbase;

    wheelLoads = cell(2, 2);
    wheelLoads{1, 1} = W_static_fL + deltaW_scf + deltaW_sff + deltaW_uf + deltaW_f_jacking - deltaLong / 2;
    wheelLoads{1, 2} = W_static_fR - deltaW_scf - deltaW_sff - deltaW_uf - deltaW_f_jacking - deltaLong / 2;
    wheelLoads{2, 1} = W_static_rL + deltaW_scr + deltaW_sfr + deltaW_ur + deltaW_r_jacking + deltaLong / 2;
    wheelLoads{2, 2} = W_static_rR - deltaW_scr - deltaW_sfr - deltaW_ur - deltaW_r_jacking + deltaLong / 2;
    frontLoad = wheelLoads{1, 1} + wheelLoads{1, 2};
    rearLoad = wheelLoads{2, 1} + wheelLoads{2, 2};
    totalLoad = frontLoad + rearLoad;
    leftLoad = wheelLoads{1, 1} + wheelLoads{2, 1};
    rightLoad = totalLoad - leftLoad;

    Fy_front = m_tot * a_y * params.weightDistF;
    Fy_rear = m_tot * a_y * (1 - params.weightDistF);
    frontSA = zeros(1, n);
    rearSA = zeros(1, n);
    for i = 1:n
        frontSA(i) = findSlip(wheelLoads{1, 1}(i), frontLoad(i), Fy_front(i), IA1(i), IA2(i), toeEff, P, L) * pi / 180;
        rearSA(i) = findSlip(wheelLoads{2, 1}(i), rearLoad(i), Fy_rear(i), IA3(i), IA4(i), params.toeR, P, L) * pi / 180;
    end

    parasiticDrag = a_y * m_tot .* sin(rearSA) + 0.5 * a_y * m_tot .* sin(frontSA - rearSA) + 0.02 * params.W_tot;
    Fx_rear = zeros(1, n);
    Fx_front = zeros(1, n);
    for i = 1:n
        if a_x(i) < 0
            Fx_rear(i) = (a_x(i) * m_tot + parasiticDrag(i) + DF(V(i))) * 0.3;
            Fx_front(i) = (a_x(i) * m_tot + parasiticDrag(i) + DF(V(i))) * 0.7;
        else
            Fx_rear(i) = a_x(i) * m_tot + parasiticDrag(i) + DF(V(i));
        end
    end
    Fy_front = sqrt(Fy_front.^2 + Fx_front.^2);
    Fy_rear = sqrt(Fy_rear.^2 + Fx_rear.^2);

    Mu = 0.03 * (leftLoad - rightLoad) .* params.TF / 2;
    deltaF_rollingResistance = Mu ./ params.wheelbase;
    for i = 1:n
        frontSA(i) = findSlip(wheelLoads{1, 1}(i), frontLoad(i), Fy_front(i), IA1(i), IA2(i), toeEff, P, L) * pi / 180;
        rearSA(i) = findSlip(wheelLoads{2, 1}(i), rearLoad(i), Fy_rear(i), IA3(i), IA4(i), params.toeR, P, L) * pi / 180;
    end

    MZ_fo = zeros(1, n);
    MZ_fi = zeros(1, n);
    MZ_ro = zeros(1, n);
    MZ_ri = zeros(1, n);
    for i = 1:n
        MZ_fo(i) = pacejkaMZ(PM, LMZ, wheelLoads{1, 1}(i), IA1(i) * pi / 180, frontSA(i)) * 12;
        MZ_fi(i) = pacejkaMZ(PM, LMZ, wheelLoads{1, 2}(i), -IA2(i) * pi / 180, frontSA(i)) * 12;
        MZ_ro(i) = pacejkaMZ(PM, LMZ, wheelLoads{2, 1}(i), IA3(i) * pi / 180, rearSA(i)) * 12;
        MZ_ri(i) = pacejkaMZ(PM, LMZ, wheelLoads{2, 2}(i), -IA4(i) * pi / 180, rearSA(i)) * 12;
    end

    Mu = MZ_fi + MZ_fo + MZ_ro + MZ_ri;
    deltaF_selfAllign = Mu ./ params.wheelbase;
    Mu = (wheelLoads{1, 1} - wheelLoads{1, 2}) .* a_y / 32.2 .* sin(frontSA - rearSA) .* params.TF / 2;
    deltaF_inducedDrag = Mu ./ params.TF;
    Fy_front = Fy_front + deltaF_rollingResistance + deltaF_selfAllign + deltaF_inducedDrag;
    Fy_rear = Fy_rear - deltaF_rollingResistance - deltaF_selfAllign - deltaF_inducedDrag;

    for i = 1:n
        frontSA(i) = findSlip(wheelLoads{1, 1}(i), frontLoad(i), Fy_front(i), IA1(i), IA2(i), toeEff, P, L);
        rearSA(i) = findSlip(wheelLoads{2, 1}(i), rearLoad(i), Fy_rear(i), IA3(i), IA4(i), params.toeR, P, L);
    end

    result.g = a_y / 32.2;
    result.frontSA = frontSA;
    result.rearSA = rearSA;
    result.minInsideWheelLoad = min([wheelLoads{1, 1}; wheelLoads{1, 2}; ...
                                     wheelLoads{2, 1}; wheelLoads{2, 2}], [], 1);
end

function metrics = evaluateBalanceParameterCandidates(candidateValues, parameterName, radii, baseParams, ...
                                                       P, L, PM, LMZ, sampleCount, slipCapDeg, ...
                                                       targetSlipGap, saturationPenalty, wheelLiftPenalty)
    candidateCount = numel(candidateValues);
    metrics.objective = inf(size(candidateValues));
    metrics.rmsSlipGapError = inf(size(candidateValues));
    metrics.saturationFraction = zeros(size(candidateValues));
    metrics.wheelLiftFraction = zeros(size(candidateValues));

    for candidateIdx = 1:candidateCount
        squaredErrorSum = 0;
        validPointCount = 0;
        saturatedPointCount = 0;
        wheelLiftPointCount = 0;
        totalPointCount = 0;

        for radiusIdx = 1:numel(radii)
            candidateParams = baseParams;
            candidateParams.(char(parameterName)) = candidateValues(candidateIdx);
            candidateParams.r_corner = radii(radiusIdx);
            candidateResult = evaluateVehicleBalance(candidateParams, P, L, PM, LMZ, sampleCount);

            slipGap = candidateResult.frontSA - candidateResult.rearSA;
            saturated = candidateResult.frontSA >= slipCapDeg | candidateResult.rearSA >= slipCapDeg;
            wheelLift = candidateResult.minInsideWheelLoad <= 0;
            valid = isfinite(slipGap) & ~saturated & ~wheelLift;

            squaredErrorSum = squaredErrorSum + sum((slipGap(valid) - targetSlipGap).^2);
            validPointCount = validPointCount + nnz(valid);
            saturatedPointCount = saturatedPointCount + nnz(saturated);
            wheelLiftPointCount = wheelLiftPointCount + nnz(wheelLift);
            totalPointCount = totalPointCount + numel(slipGap);
        end

        if validPointCount > 0
            metrics.rmsSlipGapError(candidateIdx) = sqrt(squaredErrorSum / validPointCount);
        end
        metrics.saturationFraction(candidateIdx) = saturatedPointCount / totalPointCount;
        metrics.wheelLiftFraction(candidateIdx) = wheelLiftPointCount / totalPointCount;
        metrics.objective(candidateIdx) = metrics.rmsSlipGapError(candidateIdx) + ...
            saturationPenalty * metrics.saturationFraction(candidateIdx) + ...
            wheelLiftPenalty * metrics.wheelLiftFraction(candidateIdx);
    end
end

function value = getOverrideValue(overrides, fieldName, defaultValue)
    fieldName = char(fieldName);
    if isfield(overrides, fieldName)
        value = overrides.(fieldName);
    else
        value = defaultValue;
    end
end

function printOverrideReport(overrides, overrideSource)
    overrideNames = fieldnames(overrides);
    if isempty(overrideNames)
        fprintf("VehicleBalance_MF14: no overrides active.\n");
        return
    end

    fprintf("VehicleBalance_MF14: using overrides from %s\n", overrideSource);
    for overrideIdx = 1:numel(overrideNames)
        overrideName = overrideNames{overrideIdx};
        overrideValue = overrides.(overrideName);
        fprintf("  %s = %s\n", overrideName, formatOverrideValue(overrideValue));
    end
end

function textValue = formatOverrideValue(value)
    if isstring(value) || ischar(value)
        textValue = """" + string(value) + """";
    elseif islogical(value) && isscalar(value)
        textValue = string(mat2str(value));
    elseif isnumeric(value) && isscalar(value)
        textValue = string(num2str(value, "%.10g"));
    elseif isnumeric(value)
        textValue = string(mat2str(value));
    else
        textValue = "<" + string(class(value)) + ">";
    end
end
