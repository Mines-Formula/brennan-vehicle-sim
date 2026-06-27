clear;
clc;
close all;

addpath(genpath("Tire Modeling"));

%% MF14 Multi-Parameter Vehicle Balance Optimizer
% Standalone bounded coordinate-search optimizer. Baseline constants mirror
% VehicleBalance_MF14.m as of this file's creation. Keep the two files in
% sync when non-optimized vehicle or tire parameters change.
%
% Optimized variables:
%   1. Effective rear ARB axle roll stiffness [N*m/deg]
%   2. Sprung-mass CG height [in]
%   3. Front downforce distribution [-]
%   4. Front wheel rate [lbf/in]
%   5. Rear wheel rate [lbf/in]

outputDir = "VehicleBalance_MF14_optimization_outputs";
if ~exist(outputDir, "dir")
    mkdir(outputDir);
end

%% Fixed vehicle parameters copied from VehicleBalance_MF14.m
params = struct( ...
    'W_tot', 580, ...
    'weightDistF', 0.48, ...
    'weightDistL', 0.50, ...
    'm_uf', 38.8 / 32.2, ...
    'm_ur', 38.2 / 32.2, ...
    'wheelbase', 60.5, ...
    'TF', 48, ...
    'TR', 48, ...
    'r_l', 7.875, ...
    'toeF', 0, ...
    'toeR', 0, ...
    'camberF', -1.25, ...
    'camberR', -1.25, ...
    'castor', 3.99, ...
    'KPI', 7.61, ...
    'CL', 3.71, ...
    'CLCD', 3.71 / 1.71, ...
    'rc_zf', 2.329, ...
    'rc_zr', 2.644, ...
    'kRoll_f_arb', 0);

P = [250, 1.4, 2.4, -0.25, 3, -0.1, -1.5, 0, 0, -30.5, 1.15, 1, 0, 0, -0.128, 0, 0, 0, 1.43];
L = [0.62, 1, 1, 1, 1, 1, 1, 1];
PM = [250, 2.5, 0.16, 0.12, 0, -0.065, -0.8, 0, 0, 4.8, 1.8, 0, 0, 0, 0.2, -0.01, 0, 0.4, 0, -0.045, 250];
LMZ = [0.62, 1, 1, 1, 1, 1, 1, 1];

%% Optimization definition
variableNames = ["kRoll_r_arb", "sprung_z", "DFDistF", "kWheel_f", "kWheel_r"];
variableLabels = ["Rear ARB [N*m/deg]", "Sprung CG height [in]", ...
                  "Front downforce distribution", "Front wheel rate [lbf/in]", ...
                  "Rear wheel rate [lbf/in]"];

x0 = [100, 12.35, 0.43, 295, 273];
lowerBounds = [0, 9, 0.30, 200, 200];
upperBounds = [600, 16, 0.55, 400, 400];
initialSteps = [100, 1, 0.025, 25, 25];
minimumSteps = [12.5, 0.125, 0.003125, 3.125, 3.125];
maxIterations = 40;

optimizationRadii = [10, 20, 30, 50]; % m
optimizationG = [1.25, 1.50, 1.75, 2.00];
targetSlipGap = 0.10; % deg; slight understeer
slipCapDeg = 9.999;
saturationPenalty = 4;
wheelLiftPenalty = 4;

objectiveFunction = @(x) vehicleBalanceObjective( ...
    x, params, optimizationRadii, optimizationG, targetSlipGap, slipCapDeg, ...
    saturationPenalty, wheelLiftPenalty, P, L, PM, LMZ);

%% Bounded coordinate search
[baselineObjective, baselineMetrics] = objectiveFunction(x0);
bestX = x0;
bestObjective = baselineObjective;
steps = initialSteps;
iteration = 0;
evaluationCount = 1;
historyIteration = 0;
historyObjective = bestObjective;
historyX = bestX;

fprintf("MF14 balance optimization\n");
fprintf("Baseline objective: %.4f deg-equivalent\n", baselineObjective);

while iteration < maxIterations && any(steps >= minimumSteps)
    iteration = iteration + 1;
    iterationImproved = false;

    for variableIdx = 1:numel(bestX)
        for direction = [-1, 1]
            candidateX = bestX;
            candidateX(variableIdx) = min(upperBounds(variableIdx), ...
                max(lowerBounds(variableIdx), bestX(variableIdx) + direction * steps(variableIdx)));
            if candidateX(variableIdx) == bestX(variableIdx)
                continue
            end

            candidateObjective = objectiveFunction(candidateX);
            evaluationCount = evaluationCount + 1;
            if candidateObjective < bestObjective
                bestX = candidateX;
                bestObjective = candidateObjective;
                iterationImproved = true;
            end
        end
    end

    if ~iterationImproved
        steps = steps / 2;
    end

    historyIteration(end + 1) = iteration; %#ok<SAGROW>
    historyObjective(end + 1) = bestObjective; %#ok<SAGROW>
    historyX(end + 1, :) = bestX; %#ok<SAGROW>
    fprintf("Iteration %2d: objective %.4f, x = %s\n", ...
            iteration, bestObjective, mat2str(bestX, 5));
end

[bestObjective, bestMetrics] = objectiveFunction(bestX);
totalMass = params.W_tot / 32.2;
sprungMass = totalMass - params.m_uf - params.m_ur;
optimalTotalCG = (bestX(2) * sprungMass + params.r_l * (params.m_uf + params.m_ur)) / totalMass;

fprintf("\nOptimal bounded solution after %d evaluations:\n", evaluationCount);
for variableIdx = 1:numel(bestX)
    fprintf("  %-30s %.6g\n", variableLabels(variableIdx), bestX(variableIdx));
end
fprintf("  %-30s %.6g\n", "Calculated total CG height [in]", optimalTotalCG);
fprintf("  RMS slip-gap error [deg]     %.4f\n", bestMetrics.rmsSlipGapError);
fprintf("  Saturated points             %.1f%%\n", 100 * bestMetrics.saturationFraction);
fprintf("  Wheel-lift points            %.1f%%\n", 100 * bestMetrics.wheelLiftFraction);
fprintf("  Objective improvement        %.1f%%\n", 100 * (baselineObjective - bestObjective) / baselineObjective);

%% Save results
result = struct();
for variableIdx = 1:numel(bestX)
    result.(variableNames(variableIdx)) = bestX(variableIdx);
end
result.totalCG_z = optimalTotalCG;
result.objective = bestObjective;
result.rmsSlipGapError = bestMetrics.rmsSlipGapError;
result.saturationFraction = bestMetrics.saturationFraction;
result.wheelLiftFraction = bestMetrics.wheelLiftFraction;
result.targetSlipGap = targetSlipGap;
result.optimizationRadii = optimizationRadii;
result.optimizationG = optimizationG;
result.lowerBounds = lowerBounds;
result.upperBounds = upperBounds;
result.evaluationCount = evaluationCount;

resultJson = jsonencode(result, PrettyPrint=true);
resultFile = fullfile(outputDir, "optimal_vehicle_balance.json");
fileId = fopen(resultFile, "w");
assert(fileId >= 0, "Unable to open optimization result file");
cleanupFile = onCleanup(@() fclose(fileId));
fprintf(fileId, "%s\n", resultJson);
clear cleanupFile;

historyTable = array2table([historyIteration(:), historyObjective(:), historyX], ...
    'VariableNames', ["Iteration", "Objective", variableNames]);
writetable(historyTable, fullfile(outputDir, "optimization_history.csv"));

figure("Name", "MF14 Multi-Parameter Balance Optimization", "NumberTitle", "off");
subplot(2, 1, 1);
plot(historyIteration, historyObjective, "o-", "LineWidth", 1.5);
grid on;
xlabel("Iteration");
ylabel("Objective [deg-equivalent]");
title("Vehicle Balance Optimization Convergence");

subplot(2, 1, 2);
normalizedHistory = (historyX - lowerBounds) ./ (upperBounds - lowerBounds);
plot(historyIteration, normalizedHistory, "LineWidth", 1.3);
grid on;
ylim([0, 1]);
xlabel("Iteration");
ylabel("Normalized position within bounds");
legend(variableLabels, "Location", "eastoutside");
subtitle("A value at 0 or 1 indicates that the solution reached a specified bound.");
saveas(gcf, fullfile(outputDir, "optimization_convergence.png"));

%% CG height sensitivity: maximum usable lateral acceleration
cgSweepValues = 9:0.5:16; % sprung-mass CG height [in]
accelerationBracketG = 0.75:0.10:3.00;
bisectionIterations = 10;
maxLateralG = nan(numel(cgSweepValues), numel(optimizationRadii));
rangeCensored = false(size(maxLateralG));

cgSweepParams = params;
cgSweepParams.kRoll_r_arb = bestX(1);
cgSweepParams.DFDistF = bestX(3);
cgSweepParams.kWheel_f = bestX(4);
cgSweepParams.kWheel_r = bestX(5);

for cgIdx = 1:numel(cgSweepValues)
    cgSweepParams.sprung_z = cgSweepValues(cgIdx);
    for radiusIdx = 1:numel(optimizationRadii)
        cgSweepParams.r_corner = optimizationRadii(radiusIdx);
        [maxLateralG(cgIdx, radiusIdx), rangeCensored(cgIdx, radiusIdx)] = ...
            findMaxUsableLateralG(cgSweepParams, accelerationBracketG, bisectionIterations, ...
                                  slipCapDeg, P, L, PM, LMZ);
    end
end

% Use radii with resolved limits throughout the sweep. The 50 m series can
% be range-censored and is therefore shown but excluded from this aggregate.
fitRadiusMask = optimizationRadii <= 30 & ~any(rangeCensored, 1);
meanMaxLateralG = mean(maxLateralG(:, fitRadiusMask), 2, "omitnan");
baselineMeanMaxG = interp1(cgSweepValues, meanMaxLateralG, x0(2), "linear");
improvementPercent = 100 * (meanMaxLateralG - baselineMeanMaxG) / baselineMeanMaxG;
fitCoefficients = polyfit(cgSweepValues, meanMaxLateralG', 1);
bestFitMaxG = polyval(fitCoefficients, cgSweepValues);
fitResidual = meanMaxLateralG' - bestFitMaxG;
fitR2 = 1 - sum(fitResidual.^2) / sum((meanMaxLateralG' - mean(meanMaxLateralG)).^2);

fprintf("CG sensitivity best fit: mean max g = %.6f %+.6f * sprung_z, R^2 = %.4f\n", ...
        fitCoefficients(2), fitCoefficients(1), fitR2);

figure("Name", "CG Height vs Maximum Lateral Acceleration", "NumberTitle", "off");
subplot(2, 1, 1);
radiusLines = plot(cgSweepValues, maxLateralG, "o-", "LineWidth", 1.2);
hold on;
meanLine = plot(cgSweepValues, meanMaxLateralG, "k-o", "LineWidth", 2.4);
fitLine = plot(cgSweepValues, bestFitMaxG, "k--", "LineWidth", 2);
baselineLine = xline(x0(2), "--", "Baseline CG");
optimizedLine = xline(bestX(2), ":", "Optimized CG");
grid on;
ylabel("Maximum usable lateral acceleration [g]");
title("CG Height Effect on Modeled Tire-Limited Lateral Acceleration");
legend([radiusLines(:); meanLine; fitLine; baselineLine; optimizedLine], ...
       [compose("%g m radius", optimizationRadii), "Mean (resolved 10-30 m)", ...
        sprintf("Linear fit: y = %.4f %+.4fx, R^2 = %.3f", fitCoefficients(2), fitCoefficients(1), fitR2), ...
        "Baseline CG", "Optimized CG"], ...
       "Location", "eastoutside");
subtitle(sprintf("Bisection-refined limit at %.3g deg axle slip or wheel unloading; range-censored radii excluded from mean.", slipCapDeg));
hold off;

subplot(2, 1, 2);
plot(cgSweepValues, improvementPercent, "o-", "LineWidth", 1.8);
yline(0, "--");
xline(x0(2), "--", "Baseline CG");
xline(bestX(2), ":", "Optimized CG");
grid on;
xlabel("Sprung-mass CG height [in]");
ylabel("Mean max-g improvement [%]");
subtitle(sprintf("Improvement relative to the %.2f in baseline sprung CG.", x0(2)));
saveas(gcf, fullfile(outputDir, "cg_height_vs_max_lateral_acceleration.png"));

cgSensitivityTable = array2table([cgSweepValues(:), maxLateralG, meanMaxLateralG, bestFitMaxG(:), improvementPercent], ...
    'VariableNames', ["sprung_z", compose("maxG_%gm", optimizationRadii), "meanResolvedMaxG", ...
                      "linearBestFitMaxG", "improvementPercent"]);
writetable(cgSensitivityTable, fullfile(outputDir, "cg_height_sensitivity.csv"));

%% Local functions
function [maxUsableG, rangeCensored] = findMaxUsableLateralG(params, bracketG, ...
        bisectionIterations, slipCapDeg, P, L, PM, LMZ)
    params.a_y = bracketG * 32.2;
    params.a_x = zeros(size(bracketG));
    bracketResult = evaluateBalanceCase(params, P, L, PM, LMZ);
    invalid = bracketResult.frontSA >= slipCapDeg | bracketResult.rearSA >= slipCapDeg | ...
              bracketResult.minWheelLoad <= 0 | ~isfinite(bracketResult.frontSA) | ...
              ~isfinite(bracketResult.rearSA);
    firstInvalidIdx = find(invalid, 1, "first");

    if isempty(firstInvalidIdx)
        maxUsableG = NaN;
        rangeCensored = true;
        return
    elseif firstInvalidIdx == 1
        maxUsableG = NaN;
        rangeCensored = false;
        return
    end

    lowerG = bracketG(firstInvalidIdx - 1);
    upperG = bracketG(firstInvalidIdx);
    for iterationIdx = 1:bisectionIterations
        midpointG = (lowerG + upperG) / 2;
        params.a_y = midpointG * 32.2;
        params.a_x = 0;
        midpointResult = evaluateBalanceCase(params, P, L, PM, LMZ);
        midpointInvalid = midpointResult.frontSA >= slipCapDeg | ...
                          midpointResult.rearSA >= slipCapDeg | ...
                          midpointResult.minWheelLoad <= 0 | ...
                          ~isfinite(midpointResult.frontSA) | ~isfinite(midpointResult.rearSA);
        if midpointInvalid
            upperG = midpointG;
        else
            lowerG = midpointG;
        end
    end
    maxUsableG = lowerG;
    rangeCensored = false;
end

function [objective, metrics] = vehicleBalanceObjective(x, params, radii, gValues, ...
        targetSlipGap, slipCapDeg, saturationPenalty, wheelLiftPenalty, P, L, PM, LMZ)
    params.kRoll_r_arb = x(1);
    params.sprung_z = x(2);
    params.DFDistF = x(3);
    params.kWheel_f = x(4);
    params.kWheel_r = x(5);
    params.a_y = gValues * 32.2;
    params.a_x = zeros(size(gValues));

    squaredErrorSum = 0;
    validPointCount = 0;
    saturatedPointCount = 0;
    wheelLiftPointCount = 0;
    totalPointCount = 0;

    for radiusIdx = 1:numel(radii)
        params.r_corner = radii(radiusIdx);
        modelResult = evaluateBalanceCase(params, P, L, PM, LMZ);
        slipGap = modelResult.frontSA - modelResult.rearSA;
        saturated = modelResult.frontSA >= slipCapDeg | modelResult.rearSA >= slipCapDeg;
        wheelLift = modelResult.minWheelLoad <= 0;
        valid = isfinite(slipGap) & ~saturated & ~wheelLift;

        squaredErrorSum = squaredErrorSum + sum((slipGap(valid) - targetSlipGap).^2);
        validPointCount = validPointCount + nnz(valid);
        saturatedPointCount = saturatedPointCount + nnz(saturated);
        wheelLiftPointCount = wheelLiftPointCount + nnz(wheelLift);
        totalPointCount = totalPointCount + numel(slipGap);
    end

    if validPointCount == 0
        rmsSlipGapError = inf;
    else
        rmsSlipGapError = sqrt(squaredErrorSum / validPointCount);
    end
    saturationFraction = saturatedPointCount / totalPointCount;
    wheelLiftFraction = wheelLiftPointCount / totalPointCount;
    objective = rmsSlipGapError + saturationPenalty * saturationFraction + ...
        wheelLiftPenalty * wheelLiftFraction;

    metrics = struct('rmsSlipGapError', rmsSlipGapError, ...
                     'saturationFraction', saturationFraction, ...
                     'wheelLiftFraction', wheelLiftFraction);
end

function result = evaluateBalanceCase(params, P, L, PM, LMZ)
    a_y = params.a_y;
    a_x = params.a_x;
    n = numel(a_y);
    m_tot = params.W_tot / 32.2;
    m_s = m_tot - params.m_uf - params.m_ur;

    delta1 = atan(params.wheelbase ./ (params.r_corner / 0.0254 + params.TF / 2)) * 180 / pi;
    delta2 = -delta1 * (1 + 0.002079275 * delta1) + 2 * params.toeF;
    delta2Ackerman = -atan(params.wheelbase ./ (params.r_corner / 0.0254 - params.TF / 2)) * 180 / pi;
    toeEff = (delta2 - delta2Ackerman) / 2;
    V = sqrt(params.r_corner .* a_y / 32.2 * 9.81);
    LF = @(speed) 0.5 * 1.225 * speed.^2 * params.CL * 1.08 * 0.224809;
    DF = @(speed) LF(speed) / params.CLCD;

    kRoll_f = params.kWheel_f * params.TF^2 * tand(1) / 2 * 0.113 + params.kRoll_f_arb;
    kRoll_r = params.kWheel_r * params.TR^2 * tand(1) / 2 * 0.113 + params.kRoll_r_arb;
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
    deltaW_sff = a_y * m_s * params.weightDistF * params.rc_zf / params.TF;
    deltaW_sfr = a_y * m_s * (1 - params.weightDistF) * params.rc_zr / params.TR;
    deltaW_scf = kRoll_f / (kRoll_r + kRoll_f) * rollCouple / params.TF;
    deltaW_scr = kRoll_r / (kRoll_r + kRoll_f) * rollCouple / params.TR;
    deltaW_jacking = 7 / 20 * delta1 / 2;
    CG_z = (params.sprung_z * m_s + params.r_l * (params.m_uf + params.m_ur)) / m_tot;
    deltaLong = a_x * m_tot * CG_z / params.wheelbase;

    wheelLoads = cell(2, 2);
    wheelLoads{1, 1} = W_static_fL + deltaW_scf + deltaW_sff + deltaW_uf - deltaW_jacking - deltaLong / 2;
    wheelLoads{1, 2} = W_static_fR - deltaW_scf - deltaW_sff - deltaW_uf + deltaW_jacking - deltaLong / 2;
    wheelLoads{2, 1} = W_static_rL + deltaW_scr + deltaW_sfr + deltaW_ur + deltaW_jacking + deltaLong / 2;
    wheelLoads{2, 2} = W_static_rR - deltaW_scr - deltaW_sfr - deltaW_ur - deltaW_jacking + deltaLong / 2;
    frontLoad = wheelLoads{1, 1} + wheelLoads{1, 2};
    rearLoad = wheelLoads{2, 1} + wheelLoads{2, 2};
    totalLoad = frontLoad + rearLoad;
    leftLoad = wheelLoads{1, 1} + wheelLoads{2, 1};
    rightLoad = totalLoad - leftLoad;

    Fy_front = m_tot * a_y * params.weightDistF;
    Fy_rear = m_tot * a_y * (1 - params.weightDistF);
    frontSA = zeros(1, n);
    rearSA = zeros(1, n);
    for pointIdx = 1:n
        frontSA(pointIdx) = findSlip(wheelLoads{1, 1}(pointIdx), frontLoad(pointIdx), Fy_front(pointIdx), IA1(pointIdx), IA2(pointIdx), toeEff, P, L) * pi / 180;
        rearSA(pointIdx) = findSlip(wheelLoads{2, 1}(pointIdx), rearLoad(pointIdx), Fy_rear(pointIdx), IA3(pointIdx), IA4(pointIdx), params.toeR, P, L) * pi / 180;
    end

    parasiticDrag = a_y * m_tot .* sin(rearSA) + 0.5 * a_y * m_tot .* sin(frontSA - rearSA) + 0.02 * params.W_tot;
    Fx_rear = a_x * m_tot + parasiticDrag + DF(V);
    Fx_front = zeros(1, n);
    braking = a_x < 0;
    Fx_rear(braking) = (a_x(braking) * m_tot + parasiticDrag(braking) + DF(V(braking))) * 0.3;
    Fx_front(braking) = (a_x(braking) * m_tot + parasiticDrag(braking) + DF(V(braking))) * 0.7;
    Fy_front = hypot(Fy_front, Fx_front);
    Fy_rear = hypot(Fy_rear, Fx_rear);

    deltaF_rollingResistance = 0.03 * (leftLoad - rightLoad) * params.TF / 2 / params.wheelbase;
    for pointIdx = 1:n
        frontSA(pointIdx) = findSlip(wheelLoads{1, 1}(pointIdx), frontLoad(pointIdx), Fy_front(pointIdx), IA1(pointIdx), IA2(pointIdx), toeEff, P, L) * pi / 180;
        rearSA(pointIdx) = findSlip(wheelLoads{2, 1}(pointIdx), rearLoad(pointIdx), Fy_rear(pointIdx), IA3(pointIdx), IA4(pointIdx), params.toeR, P, L) * pi / 180;
    end

    MZ_fo = zeros(1, n); MZ_fi = zeros(1, n); MZ_ro = zeros(1, n); MZ_ri = zeros(1, n);
    for pointIdx = 1:n
        MZ_fo(pointIdx) = pacejkaMZ(PM, LMZ, wheelLoads{1, 1}(pointIdx), IA1(pointIdx) * pi / 180, frontSA(pointIdx)) * 12;
        MZ_fi(pointIdx) = pacejkaMZ(PM, LMZ, wheelLoads{1, 2}(pointIdx), -IA2(pointIdx) * pi / 180, frontSA(pointIdx)) * 12;
        MZ_ro(pointIdx) = pacejkaMZ(PM, LMZ, wheelLoads{2, 1}(pointIdx), IA3(pointIdx) * pi / 180, rearSA(pointIdx)) * 12;
        MZ_ri(pointIdx) = pacejkaMZ(PM, LMZ, wheelLoads{2, 2}(pointIdx), -IA4(pointIdx) * pi / 180, rearSA(pointIdx)) * 12;
    end
    deltaF_selfAlign = (MZ_fi + MZ_fo + MZ_ro + MZ_ri) / params.wheelbase;
    inducedMoment = (wheelLoads{1, 1} - wheelLoads{1, 2}) .* a_y / 32.2 .* sin(frontSA - rearSA) * params.TF / 2;
    deltaF_inducedDrag = inducedMoment / params.TF;
    Fy_front = Fy_front + deltaF_rollingResistance + deltaF_selfAlign + deltaF_inducedDrag;
    Fy_rear = Fy_rear - deltaF_rollingResistance - deltaF_selfAlign - deltaF_inducedDrag;

    for pointIdx = 1:n
        frontSA(pointIdx) = findSlip(wheelLoads{1, 1}(pointIdx), frontLoad(pointIdx), Fy_front(pointIdx), IA1(pointIdx), IA2(pointIdx), toeEff, P, L);
        rearSA(pointIdx) = findSlip(wheelLoads{2, 1}(pointIdx), rearLoad(pointIdx), Fy_rear(pointIdx), IA3(pointIdx), IA4(pointIdx), params.toeR, P, L);
    end

    result.frontSA = frontSA;
    result.rearSA = rearSA;
    result.minWheelLoad = min([wheelLoads{1, 1}; wheelLoads{1, 2}; wheelLoads{2, 1}; wheelLoads{2, 2}], [], 1);
end
