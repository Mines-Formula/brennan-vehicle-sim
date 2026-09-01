%% MF12 Vehicle Parameters
% This parameter script is sourced by a balance-analysis driver. It
% populates `setup` and `steeringSetups` in the caller workspace; it does
% not clear existing variables or run the vehicle model.
%
% The values below are transcribed from the legacy VehicleBalance_MF12.m
% setup block. MF12 used a fixed 35 lbf downforce at its 10 m reference
% corner, so CL is an equivalent value at approximately 1 g and 10 m.
%
% ARRAY SWEEPS: every numeric field in `setup` is sweepable. A scalar runs
% one value; a vector runs every value. Multiple vector fields produce a
% Cartesian sweep in the main driver.

setup = struct();

%% Mass and Weight Distribution
setup.W_tot = 595;             % total car + driver weight [lbf]
setup.weightDistF = 0.49;      % static front weight fraction [-]
setup.weightDistL = 0.50;      % static left weight fraction [-]
setup.W_unsprungF = 37.5;      % total front unsprung weight [lbf]
setup.W_unsprungR = 40.5;      % total rear unsprung weight [lbf]

%% Vehicle Geometry
setup.wheelbase = 60.5;        % wheelbase [in]
setup.TF = 48;                 % front track [in]
setup.TR = 48;                 % rear track [in]
setup.r_l = 7.875;             % loaded tire radius [in]
setup.sprung_z = 13;           % sprung-mass CG height [in]
setup.unsprung_z = 7.875;      % unsprung-mass CG height [in]

%% Alignment and Steering Geometry
setup.toeF = 0;                % static front toe [deg]
setup.toeR = 0;                % static rear toe [deg]
setup.camberF = 0;             % legacy MF12 used zero modeled camber [deg]
setup.camberR = 0;             % legacy MF12 used zero modeled camber [deg]
setup.castor = 4;              % caster for dynamic camber [deg]
setup.KPI = 8;                 % kingpin inclination [deg]

%% Maneuver Geometry
setup.r_corner = 10;           % centerline corner radius [m]

%% Aerodynamics
setup.CL = 2.36;               % equivalent to approximately 35 lbf at 10 m/1 g
setup.CD = setup.CL / 2;       % legacy MF12 used drag = downforce / 2
setup.DFDistF = 0.40;          % front downforce fraction [-]
setup.rhoAir = 1.225;          % air density [kg/m^3]
setup.aeroArea = 1.08;         % reference area [m^2]

%% Suspension
setup.rc_zf = 2.3;             % front roll-center height [in]
setup.rc_zr = 2.7;             % rear roll-center height [in]
setup.kWheel_f = 455;          % front wheel rate [lbf/in]
setup.kWheel_r = 320;          % rear wheel rate [lbf/in]
setup.kRoll_f_arb = 0;         % effective front ARB stiffness [N*m/deg]
setup.kRoll_r_arb = 400;       % effective rear ARB stiffness [N*m/deg]

%% Kinematic and Balance Assumptions
setup.ackermannCorrection = 0.002079275; % inside-steer correction [1/deg]
setup.jackingLoadAt20Deg = 3;            % steering jacking at 20 deg [lbf]
setup.frontHeaveCamberGain = 0;           % legacy MF12 modeled no heave camber [deg/in]
setup.rearHeaveCamberGain = 0;            % legacy MF12 modeled no heave camber [deg/in]
setup.frontRollCamberGain = 0;            % legacy MF12 modeled no roll camber [deg/deg]
setup.rearRollCamberGain = 0;             % legacy MF12 modeled no roll camber [deg/deg]
setup.rollingResistanceCoeff = 0.02;      % lateral imbalance coefficient [-]
setup.parasiticDragCoeff = 0.02;           % straight-line tire drag fraction [-]
setup.brakeBiasF = 0.70;                   % front braking fraction [-]

%% Conversions and Model Constants
setup.g_ftps2 = 32.2;                    % [ft/s^2]
setup.g_mps2 = 9.81;                     % [m/s^2]
setup.metersPerInch = 0.0254;             % [m/in]
setup.newtonsToLbf = 0.224809;            % [lbf/N]
setup.rollStiffnessConversion = 0.113;    % legacy wheel-rate conversion

%% Steering-Effort Comparison Setups
steeringSetups = struct( ...
    "label", {"MF13", "MF12", "MF11"}, ...
    "scrubRadius", {0.579, 0.539, 0.770}, ...
    "KPI", {7.6, 8.0, 5.34}, ...
    "steerAngle", {10, 14, 10}, ...
    "castor", {3.94, 4.0, 2.0}, ...
    "trail", {0.552, 0.752, 0.475}, ...
    "steeringArm", {2.95, 3.15, 2.95}, ...
    "pinionRadius", {0.625, 0.625, 0.625}, ...
    "steeringWheelDiameter", {8.5, 8.5, 8.5});
