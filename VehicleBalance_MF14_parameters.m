%% MF14 Vehicle Parameters
%
% ARRAY SWEEPS: every numeric field in `setup` remains sweepable. Replace a
% scalar with a vector to generate cases in the main script.

setup = struct();

%% Mass and Weight Distribution
setup.W_tot = 580;             % total car + driver weight [lbf]
setup.weightDistF = 0.48;      % static front weight fraction [-]
setup.weightDistL = 0.50;      % static left weight fraction [-]
setup.W_unsprungF = 37;      % unsprung front mass [lbf]
setup.W_unsprungR = 37;      % unsprung rear mass = 38.2/32.2 [lbf]

%% Vehicle Geometry
setup.wheelbase = 60.5;        % wheelbase [in]
setup.TF = 48;                 % front track [in]
setup.TR = 48;                 % rear track [in]
setup.r_l = 7.875;             % loaded tire radius [in]
setup.sprung_z = 12.35;        % sprung-mass CG height [in]
setup.unsprung_z = setup.r_l;  % unsprung CG from tire radius [in]

%% Alignment and Steering Geometry
setup.toeF = 0;                % static front toe [deg]
setup.toeR = 0;                % static rear toe [deg]
setup.camberF = -1.25;         % static front camber [deg]
setup.camberR = -1.25;         % static rear camber [deg]
setup.castor = 3.99;           % caster for dynamic camber [deg]
setup.KPI = 7.61;              % kingpin inclination [deg]

%% Maneuver Geometry
setup.r_corner = 30;           % centerline corner radius [m]

%% Aerodynamics
setup.CL = 4.1;               % positive coefficient creates downforce [-]
setup.CD = 1.8;               % drag coefficient [-]
setup.DFDistF = [0.4:0.005:0.5];          % front downforce fraction [-]
setup.rhoAir = 1.225;          % air density [kg/m^3]
setup.aeroArea = 1.08;         % reference area [m^2]

%% Suspension
setup.rc_zf = 2.5;           % front roll-center height [in]
setup.rc_zr = 2.9;           % rear roll-center height [in]
setup.kWheel_f = 295;          % front wheel rate [lbf/in]
setup.kWheel_r = 273;          % rear wheel rate [lbf/in]
setup.kRoll_f_arb = 0;         % effective front ARB stiffness [N*m/deg]
setup.kRoll_r_arb = 200;       % effective rear ARB stiffness [N*m/deg]

%% Kinematic and Balance Assumptions
setup.ackermannCorrection = 0.002079275; % inside-steer correction [1/deg]
setup.jackingLoadAt20Deg = 7;            % steering jacking at 20 deg [lbf]
setup.frontHeaveCamberGain = -1.12;      % front camber/heave [deg/in]
setup.rearHeaveCamberGain = -0.972;      % rear camber/heave [deg/in]
setup.frontRollCamberGain = 0.531;       % front camber/body roll [deg/deg]
setup.rearRollCamberGain = 0.593;        % rear camber/body roll [deg/deg]
setup.rollingResistanceCoeff = 0.03;     % lateral imbalance coefficient [-]
setup.parasiticDragCoeff = 0.02;         % straight-line tire drag fraction [-]
setup.brakeBiasF = 0.70;                 % front braking fraction [-]

%% Conversions and Model Constants
setup.g_ftps2 = 32.2;                    % [ft/s^2]
setup.g_mps2 = 9.81;                     % [m/s^2]
setup.metersPerInch = 0.0254;            % [m/in]
setup.newtonsToLbf = 0.224809;            % [lbf/N]
setup.rollStiffnessConversion = 0.113;    % legacy wheel-rate conversion

%% Steering-Effort Comparison Setups
steeringSetups = struct( ...
                        "label", {"MF13", "MF12", "MF11"}, ...
                        "scrubRadius", {0.579, 0.539, 0.770}, ...
                        "KPI", {7.6, 8.0, 5.34}, ...
                        "steerAngle", {10, 10, 10}, ...
                        "castor", {3.94, 4.0, 2.0}, ...
                        "trail", {0.552, 0.752, 0.475}, ...
                        "steeringArm", {2.95, 2.95, 2.95}, ...
                        "pinionRadius", {0.625, 0.625, 0.625}, ...
                        "steeringWheelDiameter", {8.5, 8.5, 8.5});
