clear;
clc;
close all;

scriptPath = mfilename("fullpath");
scriptDir = fileparts(scriptPath);
addpath(genpath(fullfile(scriptDir, "Tire Modeling")));

%% Vehicle Balance - MF14
% Steady-state cornering balance model. Unless noted otherwise, units are
% lbf, inches, seconds, degrees, and slugs.
%
% Vehicle values and sweep vectors are defined separately so this file is
% focused on analysis and plotting.
parameterFile = fullfile(scriptDir, "VehicleBalance_MF14_parameters.m");
assert(isfile(parameterFile), "MF14 parameter file not found: %s", parameterFile);
run(parameterFile);

%% Operating-Point Arrays (not setup sweep dimensions)
operating.a_y_g = linspace(1, 1.66, 1000);
operating.a_x_g = zeros(size(operating.a_y_g));

%% Corner-Radius Balance Heat Map
heatMap.enabled = true;
heatMap.radiusValues = [5, 7.5, 10, 12.5, 15, 20, 30, 50, 60, 70]; % [m]
heatMap.a_y_g = linspace(1, 2.25, 200); % independent map resolution [g]
heatMap.slipCapDeg = 9.999;             % findSlip saturation threshold [deg]
heatMap.neutralBandDeg = 0.05;          % dotted near-neutral contours [deg]
heatMap.speedContoursMph = 15:5:75;
heatMap.arrowColumnCount = 13;
heatMap.saturationMarkerColumnCount = 40;
heatMap.outputDir = fullfile(scriptDir, "VehicleBalance_MF14_outputs");
heatMap.outputFile = "corner_radius_balance_map.png";

%% Balance-Optimization Defaults
optimizationDefaults.radii = [10, 20, 30, 50];           % [m]
optimizationDefaults.gValues = [1.25, 1.50, 1.75, 2.00]; % [g]
optimizationDefaults.targetSlipGapDeg = 0.10;            % slight understeer
optimizationDefaults.saturationPenalty = 4;              % deg-equivalent
optimizationDefaults.wheelLiftPenalty = 4;               % deg-equivalent
optimizationDefaults.slipCapDeg = heatMap.slipCapDeg;
optimizationDefaults.outputDir = heatMap.outputDir;

%% Aero-Balance Optimization
aeroOptimization = optimizationDefaults;
aeroOptimization.enabled = true;
aeroOptimization.parameterName = "DFDistF";
aeroOptimization.parameterLabel = "front downforce distribution";
aeroOptimization.consoleLabel = "Aero balance";
aeroOptimization.figureTitle = "Optimal Aero Balance";
aeroOptimization.figureName = "Aero Balance Optimization";
aeroOptimization.coarseValues = 0.30:0.025:0.55;
aeroOptimization.fineHalfRange = 0.025;
aeroOptimization.fineStep = 0.005;
aeroOptimization.lowerBound = 0;
aeroOptimization.upperBound = 1;
aeroOptimization.displayScale = 100;
aeroOptimization.valueFormat = "%.1f%%";
aeroOptimization.xLabel = "Front downforce distribution [%]";
aeroOptimization.outputFile = "aero_balance_optimization.png";

%% Rear-ARB Balance Optimization
arbOptimization = optimizationDefaults;
arbOptimization.enabled = true;
arbOptimization.parameterName = "kRoll_r_arb";
arbOptimization.parameterLabel = "rear ARB roll stiffness";
arbOptimization.consoleLabel = "Rear ARB";
arbOptimization.figureTitle = "Optimal Rear ARB Stiffness";
arbOptimization.figureName = "Rear ARB Optimization";
arbOptimization.coarseValues = 0:50:600;                 % [N*m/deg]
arbOptimization.fineHalfRange = 50;
arbOptimization.fineStep = 10;
arbOptimization.lowerBound = 0;
arbOptimization.upperBound = inf;
arbOptimization.displayScale = 1;
arbOptimization.valueFormat = "%.0f N*m/deg";
arbOptimization.xLabel = "Rear ARB roll stiffness [N*m/deg]";
arbOptimization.outputFile = "rear_arb_optimization.png";

%% Joint Aero / Rear-ARB Solution Space
jointOptimization = optimizationDefaults;
jointOptimization.enabled = true;
jointOptimization.aeroValues = 0.36:0.01:0.50;            % front DF fraction
jointOptimization.arbValues = 0:25:600;                  % [N*m/deg]
jointOptimization.arbTuningHalfRange = 175;              % nominal +/- adjustment [N*m/deg]
jointOptimization.nominalArbCenter = 200;                % preferred center of tuning range [N*m/deg]
jointOptimization.arbCenterPenaltyPer100 = 0.25;         % deg-equivalent per 100 N*m/deg from center
jointOptimization.tuningBalanceMarginDeg = 0.05;         % required mean OS/US margin [deg]
jointOptimization.tunabilityPenalty = 4;                 % penalty when endpoints do not bracket balance
jointOptimization.tunabilityShortfallWeight = 1;         % penalty per degree short of endpoint margins
jointOptimization.figureName = "Aero and Rear ARB Solution Space";
jointOptimization.outputFile = "aero_arb_solution_space.png";
jointOptimization.objectiveOutputFile = "aero_arb_penalized_objective.png";

%% Tire Model Coefficients (not setup sweep dimensions)
tire.P = [250, 1.4, 2.4, -0.25, 3, -0.1, -1.5, 0, 0, -30.5, ...
          1.15, 1, 0, 0, -0.128, 0, 0, 0, 1.43];
tire.L = [0.62, 1, 1, 1, 1, 1, 1, 1];
tire.PM = [250, 2.5, 0.16, 0.12, 0, -0.065, -0.8, 0, 0, 4.8, ...
           1.8, 0, 0, 0, 0.2, -0.01, 0, 0.4, 0, -0.045, 250];
tire.LMZ = [0.62, 1, 1, 1, 1, 1, 1, 1];

%% Run Requested Cases
[setupCases, caseInfo] = expandSetupCases(setup);
results = evaluateVehicleBalance(setupCases(1), operating, tire);
results = repmat(results, numel(setupCases), 1);
steeringResults = cell(numel(setupCases), 1);
for caseIdx = 1:numel(setupCases)
    if caseIdx > 1
        results(caseIdx) = evaluateVehicleBalance(setupCases(caseIdx), operating, tire);
    end
    steeringResults{caseIdx} = evaluateSteeringEffort(results(caseIdx), steeringSetups);
end
plotVehicleBalance(results, steeringResults, caseInfo, steeringSetups);

[aeroOptimizationResults, aeroOptimizationFigure, ...
    aeroOptimizationTabs, aeroOptimizationTabGroup] = ...
    runBalanceParameterOptimization(setup, aeroOptimization, tire);

[arbOptimizationResults, arbOptimizationFigure, ...
    arbOptimizationTabs, arbOptimizationTabGroup] = ...
    runBalanceParameterOptimization(setup, arbOptimization, tire);

[jointOptimizationResults, jointOptimizationFigure, ...
    jointOptimizationTabs, jointOptimizationTabGroup] = ...
    runAeroArbSolutionSpace(setup, jointOptimization, tire);

cornerRadiusMaps = cell(numel(setupCases), 1);
cornerRadiusFigures = gobjects(numel(setupCases), 1);
cornerRadiusAxes = gobjects(numel(setupCases), 1);
cornerRadiusTabs = gobjects(numel(setupCases), 1);
cornerRadiusTabGroup = gobjects(1);
if heatMap.enabled
    if ~exist(heatMap.outputDir, "dir")
        mkdir(heatMap.outputDir);
    end
    cornerRadiusFigure = figure("Name", "Corner Radius Balance Maps", ...
        "NumberTitle", "off", "Position", [100, 100, 1200, 780]);
    cornerRadiusFigures(:) = cornerRadiusFigure;
    if numel(setupCases) > 1
        cornerRadiusTabGroup = uitabgroup(cornerRadiusFigure);
    end
    for caseIdx = 1:numel(setupCases)
        cornerRadiusMaps{caseIdx} = calculateCornerRadiusMap( ...
            setupCases(caseIdx), heatMap, tire);
        if numel(setupCases) > 1
            cornerRadiusTabs(caseIdx) = uitab(cornerRadiusTabGroup, ...
                "Title", caseInfo.labels(caseIdx));
            cornerRadiusTabGroup.SelectedTab = cornerRadiusTabs(caseIdx);
            cornerRadiusAxes(caseIdx) = axes(cornerRadiusTabs(caseIdx)); %#ok<LAXES>
        else
            cornerRadiusAxes(caseIdx) = axes(cornerRadiusFigure); %#ok<LAXES>
        end
        plotCornerRadiusBalanceMap(cornerRadiusAxes(caseIdx), ...
            cornerRadiusMaps{caseIdx}, heatMap, caseInfo.labels(caseIdx));
        drawnow;
        outputFile = caseOutputFile(heatMap.outputFile, caseInfo, caseIdx);
        exportCornerRadiusMap(cornerRadiusAxes(caseIdx), cornerRadiusFigure, ...
            fullfile(heatMap.outputDir, outputFile));
    end
    if numel(setupCases) > 1
        cornerRadiusTabGroup.SelectedTab = cornerRadiusTabs(1);
    end
end

%% Local Functions
function [optimizationResults, optimizationFigure, optimizationTabs, optimizationTabGroup] = ...
        runBalanceParameterOptimization(setup, config, tire)
    optimizationResults = struct([]);
    optimizationFigure = gobjects(1);
    optimizationTabs = gobjects(0);
    optimizationTabGroup = gobjects(1);
    if ~config.enabled, return; end

    parameterName = char(config.parameterName);
    assert(isfield(setup, parameterName), ...
        "Optimization parameter is not present in setup: %s", parameterName);
    if ~exist(config.outputDir, "dir")
        mkdir(config.outputDir);
    end

    baseSetup = setup;
    configuredValues = baseSetup.(parameterName);
    baseSetup.(parameterName) = configuredValues(1);
    [baseCases, caseInfo] = expandSetupCases(baseSetup);
    optimizationResults = optimizeBalanceParameter(baseCases(1), config, tire);
    optimizationResults = repmat(optimizationResults, numel(baseCases), 1);
    optimizationTabs = gobjects(numel(baseCases), 1);
    optimizationFigure = figure("Name", config.figureName, "NumberTitle", "off", ...
        "Position", [150, 100, 1100, 800]);
    if numel(baseCases) > 1
        optimizationTabGroup = uitabgroup(optimizationFigure);
    end

    for caseIdx = 1:numel(baseCases)
        if caseIdx > 1
            optimizationResults(caseIdx) = ...
                optimizeBalanceParameter(baseCases(caseIdx), config, tire);
        end
        if numel(baseCases) > 1
            optimizationTabs(caseIdx) = uitab(optimizationTabGroup, ...
                "Title", caseInfo.labels(caseIdx));
            optimizationTabGroup.SelectedTab = optimizationTabs(caseIdx);
            plotParent = optimizationTabs(caseIdx);
        else
            plotParent = optimizationFigure;
        end
        optimizationLayout = plotBalanceParameterOptimization( ...
            plotParent, optimizationResults(caseIdx), ...
            config, caseInfo.labels(caseIdx));
        drawnow;
        outputFile = caseOutputFile(config.outputFile, caseInfo, caseIdx);
        exportgraphics(optimizationLayout, ...
            fullfile(config.outputDir, outputFile), "Resolution", 200);
    end
    if numel(baseCases) > 1
        optimizationTabGroup.SelectedTab = optimizationTabs(1);
    end
end

function [solutionResults, solutionFigure, solutionTabs, solutionTabGroup] = ...
        runAeroArbSolutionSpace(setup, config, tire)
    solutionResults = struct([]);
    solutionFigure = gobjects(1);
    solutionTabs = gobjects(0);
    solutionTabGroup = gobjects(1);
    if ~config.enabled, return; end

    assert(isfield(setup, "DFDistF") && isfield(setup, "kRoll_r_arb"), ...
        "Joint solution space requires setup.DFDistF and setup.kRoll_r_arb.");
    if ~exist(config.outputDir, "dir")
        mkdir(config.outputDir);
    end

    baseSetup = setup;
    baseSetup.DFDistF = baseSetup.DFDistF(1);
    baseSetup.kRoll_r_arb = baseSetup.kRoll_r_arb(1);
    [baseCases, caseInfo] = expandSetupCases(baseSetup);
    solutionResults = calculateAeroArbSolutionSpace(baseCases(1), config, tire);
    solutionResults = repmat(solutionResults, numel(baseCases), 1);
    solutionTabs = gobjects(numel(baseCases), 1);
    solutionFigure = figure("Name", config.figureName, "NumberTitle", "off", ...
        "Position", [100, 75, 1350, 900]);
    if numel(baseCases) > 1
        solutionTabGroup = uitabgroup(solutionFigure);
    end

    for caseIdx = 1:numel(baseCases)
        if caseIdx > 1
            solutionResults(caseIdx) = ...
                calculateAeroArbSolutionSpace(baseCases(caseIdx), config, tire);
        end
        if numel(baseCases) > 1
            solutionTabs(caseIdx) = uitab(solutionTabGroup, ...
                "Title", caseInfo.labels(caseIdx));
            solutionTabGroup.SelectedTab = solutionTabs(caseIdx);
            plotParent = solutionTabs(caseIdx);
        else
            plotParent = solutionFigure;
        end
        solutionLayout = plotAeroArbSolutionSpace( ...
            plotParent, solutionResults(caseIdx), config, caseInfo.labels(caseIdx));
        drawnow;
        outputFile = caseOutputFile(config.outputFile, caseInfo, caseIdx);
        exportgraphics(solutionLayout, ...
            fullfile(config.outputDir, outputFile), "Resolution", 200);
        objectiveOutputFile = caseOutputFile( ...
            config.objectiveOutputFile, caseInfo, caseIdx);
        exportAeroArbObjective(solutionResults(caseIdx), config, ...
            caseInfo.labels(caseIdx), ...
            fullfile(config.outputDir, objectiveOutputFile));
    end
    if numel(baseCases) > 1
        solutionTabGroup.SelectedTab = solutionTabs(1);
    end
end

function result = calculateAeroArbSolutionSpace(baseParams, config, tire)
    aeroValues = reshape(config.aeroValues, 1, []);
    arbValues = reshape(config.arbValues, 1, []);
    baseObjective = nan(numel(arbValues), numel(aeroValues));
    rmsError = baseObjective;
    meanSlipGap = baseObjective;
    saturationFraction = baseObjective;
    wheelLiftFraction = baseObjective;
    arbCandidateConfig = config;
    arbCandidateConfig.parameterName = "kRoll_r_arb";

    for aeroIdx = 1:numel(aeroValues)
        aeroParams = baseParams;
        aeroParams.DFDistF = aeroValues(aeroIdx);
        columnMetrics = evaluateBalanceParameterCandidates( ...
            arbValues, aeroParams, arbCandidateConfig, tire);
        baseObjective(:, aeroIdx) = columnMetrics.objective(:);
        rmsError(:, aeroIdx) = columnMetrics.rmsSlipGapError(:);
        meanSlipGap(:, aeroIdx) = columnMetrics.meanSlipGap(:);
        saturationFraction(:, aeroIdx) = columnMetrics.saturationFraction(:);
        wheelLiftFraction(:, aeroIdx) = columnMetrics.wheelLiftFraction(:);
    end

    [objective, tunable, tuningLowGap, tuningHighGap, tuningPenalty] = ...
        applyArbTunabilityPenalty(baseObjective, meanSlipGap, arbValues, config);

    tunableObjective = objective;
    tunableObjective(~tunable) = inf;
    if any(tunable(:))
        [globalObjective, globalLinearIdx] = min( ...
            tunableObjective, [], "all", "linear");
    else
        warning("No aero/ARB candidate meets the configured tunability requirement.");
        [globalObjective, globalLinearIdx] = min(objective, [], "all", "linear");
    end
    [globalArbIdx, globalAeroIdx] = ind2sub(size(objective), globalLinearIdx);
    [bestObjectiveByAero, bestArbIdxByAero] = min(tunableObjective, [], 1);
    [~, bestAeroIdxByArb] = min(objective, [], 2);

    result.baseParams = baseParams;
    result.aeroValues = aeroValues;
    result.arbValues = arbValues;
    result.baseObjective = baseObjective;
    result.objective = objective;
    result.rmsSlipGapError = rmsError;
    result.meanSlipGap = meanSlipGap;
    result.saturationFraction = saturationFraction;
    result.wheelLiftFraction = wheelLiftFraction;
    result.tunable = tunable;
    result.tuningLowGap = tuningLowGap;
    result.tuningHighGap = tuningHighGap;
    result.tuningPenalty = tuningPenalty;
    result.globalObjective = globalObjective;
    result.optimalDFDistF = aeroValues(globalAeroIdx);
    result.optimalKRollRearArb = arbValues(globalArbIdx);
    result.optimalArbByAero = arbValues(bestArbIdxByAero);
    result.optimalArbByAero(~isfinite(bestObjectiveByAero)) = NaN;
    result.optimalAeroByArb = aeroValues(bestAeroIdxByArb);

    result.optimalIsTunable = tunable(globalArbIdx, globalAeroIdx);
    result.optimalTuningLowGap = tuningLowGap(globalArbIdx, globalAeroIdx);
    result.optimalTuningHighGap = tuningHighGap(globalArbIdx, globalAeroIdx);

    fprintf("Joint aero/ARB solution: %.1f%% front downforce, %.0f N*m/deg rear ARB; objective = %.3f\n", ...
        100 .* result.optimalDFDistF, ...
        result.optimalKRollRearArb, globalObjective);
    fprintf("  Preferred ARB center = %.0f N*m/deg; +/-%.0f N*m/deg endpoint mean gaps: %+.3f / %+.3f deg; tunable = %s\n", ...
        config.nominalArbCenter, ...
        config.arbTuningHalfRange, ...
        result.optimalTuningLowGap, result.optimalTuningHighGap, ...
        string(result.optimalIsTunable));
end

function [objective, tunable, lowGap, highGap, penalty] = ...
        applyArbTunabilityPenalty(baseObjective, meanSlipGap, arbValues, config)
    objective = baseObjective;
    tunable = false(size(baseObjective));
    lowGap = nan(size(baseObjective));
    highGap = nan(size(baseObjective));
    penalty = config.tunabilityPenalty .* ones(size(baseObjective));
    halfRange = config.arbTuningHalfRange;
    margin = config.tuningBalanceMarginDeg;
    arbGrid = arbValues(:);

    for aeroIdx = 1:size(baseObjective, 2)
        lowArb = arbGrid - halfRange;
        highArb = arbGrid + halfRange;
        rangeAvailable = lowArb >= min(arbGrid) & highArb <= max(arbGrid);
        lowGap(rangeAvailable, aeroIdx) = interp1(arbValues, ...
            meanSlipGap(:, aeroIdx), lowArb(rangeAvailable), "linear");
        highGap(rangeAvailable, aeroIdx) = interp1(arbValues, ...
            meanSlipGap(:, aeroIdx), highArb(rangeAvailable), "linear");

        understeerEnd = max(lowGap(:, aeroIdx), highGap(:, aeroIdx));
        oversteerEnd = min(lowGap(:, aeroIdx), highGap(:, aeroIdx));
        endpointShortfall = max(0, margin - understeerEnd) ...
            + max(0, oversteerEnd + margin);
        endpointShortfall(~rangeAvailable | ~isfinite(endpointShortfall)) = 0;
        tunable(:, aeroIdx) = rangeAvailable(:) ...
            & understeerEnd >= margin & oversteerEnd <= -margin;
        penalty(:, aeroIdx) = config.tunabilityPenalty .* ~tunable(:, aeroIdx) ...
            + config.tunabilityShortfallWeight .* endpointShortfall;
    end
    arbCenterPenalty = config.arbCenterPenaltyPer100 ...
        .* abs(arbGrid - config.nominalArbCenter) ./ 100;
    penalty = penalty + arbCenterPenalty;
    objective = objective + penalty;
end

function layout = plotAeroArbSolutionSpace(parent, result, config, caseLabel)
    layout = tiledlayout(parent, 2, 2, "TileSpacing", "compact", "Padding", "compact");
    title(layout, "Aero / Rear-ARB Balance Trade Space: " + caseLabel);
    subtitle(layout, [compose( ...
        "Global optimum: %.1f%% front aero, %.0f N*m/deg rear ARB; nominal target = +%.2f deg", ...
        100 .* result.optimalDFDistF, result.optimalKRollRearArb, ...
        config.targetSlipGapDeg), ...
        compose("Preferred nominal rear ARB = %.0f N*m/deg; tunability requires oversteer and understeer at +/-%.0f N*m/deg.", ...
        config.nominalArbCenter, config.arbTuningHalfRange)]);
    aeroPercent = 100 .* result.aeroValues;

    objectiveAxes = nexttile(layout);
    plotAeroArbObjective(objectiveAxes, result, config, caseLabel);

    rmsAxes = nexttile(layout);
    plotSolutionMetric(rmsAxes, aeroPercent, result.arbValues, ...
        result.rmsSlipGapError, "RMS Balance Error", "RMS error [deg]", parula(256));
    markSolutionOptimum(rmsAxes, result);

    saturationAxes = nexttile(layout);
    plotSolutionMetric(saturationAxes, aeroPercent, result.arbValues, ...
        100 .* result.saturationFraction, "Slip-Cap Saturation", ...
        "Saturated operating points [%]", hot(256));
    markSolutionOptimum(saturationAxes, result);

    wheelLiftAxes = nexttile(layout);
    plotSolutionMetric(wheelLiftAxes, aeroPercent, result.arbValues, ...
        100 .* result.wheelLiftFraction, "Wheel Lift", ...
        "Wheel-lift operating points [%]", hot(256));
    markSolutionOptimum(wheelLiftAxes, result);
end

function exportAeroArbObjective(result, config, caseLabel, outputPath)
    objectiveFigure = figure("Visible", "off", "Position", [100, 100, 1000, 720]);
    closeFigure = onCleanup(@() close(objectiveFigure));
    objectiveAxes = axes(objectiveFigure);
    plotAeroArbObjective(objectiveAxes, result, config, caseLabel);
    drawnow;
    exportgraphics(objectiveAxes, outputPath, "Resolution", 200);
end

function plotAeroArbObjective(objectiveAxes, result, config, caseLabel)
    aeroPercent = 100 .* result.aeroValues;
    plotSolutionMetric(objectiveAxes, aeroPercent, result.arbValues, ...
        result.objective, "Penalized objective", "Objective [deg-equivalent]", parula(256));
    title(objectiveAxes, "Penalized Aero / Rear-ARB Objective: " + caseLabel);
    subtitle(objectiveAxes, compose( ...
        "Optimum: %.1f%% front aero, %.0f N*m/deg rear ARB; preferred center = %.0f N*m/deg", ...
        100 .* result.optimalDFDistF, result.optimalKRollRearArb, ...
        config.nominalArbCenter));
    hold(objectiveAxes, "on");
    contour(objectiveAxes, aeroPercent, result.arbValues, result.objective, ...
        8, "k-", "LineWidth", 0.6, "HandleVisibility", "off");
    valleyLine = plot(objectiveAxes, aeroPercent, result.optimalArbByAero, ...
        "w-", "LineWidth", 2.2, "DisplayName", "Best ARB at each aero balance");
    optimumMarker = plot(objectiveAxes, 100 .* result.optimalDFDistF, ...
        result.optimalKRollRearArb, "p", "MarkerSize", 13, ...
        "MarkerFaceColor", "y", "MarkerEdgeColor", "k", ...
        "DisplayName", "Global optimum");
    currentMarker = plot(objectiveAxes, 100 .* result.baseParams.DFDistF, ...
        result.baseParams.kRoll_r_arb, "o", "MarkerSize", 8, ...
        "MarkerFaceColor", "w", "MarkerEdgeColor", "k", ...
        "DisplayName", "First configured setup");
    legendHandles = [valleyLine, optimumMarker, currentMarker];
    if any(result.tunable(:)) && ~all(result.tunable(:))
        [~, tunableBoundary] = contour(objectiveAxes, aeroPercent, ...
            result.arbValues, double(result.tunable), [0.5, 0.5], ...
            "w:", "LineWidth", 2, "DisplayName", compose( ...
            "+/-%.0f N*m/deg tunable region", config.arbTuningHalfRange));
        legendHandles(end + 1) = tunableBoundary;
    end
    [aeroGrid, arbGrid] = meshgrid(aeroPercent, result.arbValues);
    wheelLiftMask = result.wheelLiftFraction > 0;
    if any(wheelLiftMask(:))
        scatter(objectiveAxes, aeroGrid(wheelLiftMask), arbGrid(wheelLiftMask), ...
            22, [0.35, 0.35, 0.35], "s", "filled", ...
            "DisplayName", "Wheel lift present");
    end
    legend(objectiveAxes, legendHandles, "Location", "best");
    hold(objectiveAxes, "off");
end

function plotSolutionMetric(ax, aeroPercent, arbValues, metric, panelTitle, colorbarLabel, colorMap)
    imagesc(ax, aeroPercent, arbValues, metric);
    set(ax, "YDir", "normal");
    colormap(ax, colorMap);
    colorbarHandle = colorbar(ax);
    colorbarHandle.Label.String = colorbarLabel;
    xlabel(ax, "Front downforce distribution [%]");
    ylabel(ax, "Rear ARB roll stiffness [N*m/deg]");
    title(ax, panelTitle);
end

function markSolutionOptimum(ax, result)
    hold(ax, "on");
    plot(ax, 100 .* result.optimalDFDistF, result.optimalKRollRearArb, ...
        "p", "MarkerSize", 11, "MarkerFaceColor", "y", ...
        "MarkerEdgeColor", "k", "HandleVisibility", "off");
    hold(ax, "off");
end

function optimizationResult = optimizeBalanceParameter(baseParams, config, tire)
    coarseMetrics = evaluateBalanceParameterCandidates( ...
        config.coarseValues, baseParams, config, tire);
    [~, coarseBestIdx] = min(coarseMetrics.objective);
    coarseBestValue = config.coarseValues(coarseBestIdx);
    fineMin = max(config.lowerBound, coarseBestValue - config.fineHalfRange);
    fineMax = min(config.upperBound, coarseBestValue + config.fineHalfRange);
    fineValues = unique(fineMin:config.fineStep:fineMax);
    fineMetrics = evaluateBalanceParameterCandidates( ...
        fineValues, baseParams, config, tire);
    [bestObjective, bestIdx] = min(fineMetrics.objective);

    optimalValue = fineValues(bestIdx);
    optimizationResult.parameterName = config.parameterName;
    optimizationResult.baseParams = baseParams;
    optimizationResult.coarseValues = config.coarseValues;
    optimizationResult.coarseMetrics = coarseMetrics;
    optimizationResult.fineValues = fineValues;
    optimizationResult.fineMetrics = fineMetrics;
    optimizationResult.optimalValue = optimalValue;
    optimizationResult.bestObjective = bestObjective;
    optimizationResult.bestRmsSlipGapError = fineMetrics.rmsSlipGapError(bestIdx);
    optimizationResult.bestSaturationFraction = fineMetrics.saturationFraction(bestIdx);
    optimizationResult.bestWheelLiftFraction = fineMetrics.wheelLiftFraction(bestIdx);
    if config.parameterName == "DFDistF"
        optimizationResult.optimalDFDistF = optimalValue;
    elseif config.parameterName == "kRoll_r_arb"
        optimizationResult.optimalKRollRearArb = optimalValue;
    end

    formattedValue = sprintf(char(config.valueFormat), ...
        config.displayScale .* optimalValue);
    fprintf("%s optimizer: optimal %s = %s; objective = %.3f deg-equivalent\n", ...
        config.consoleLabel, config.parameterLabel, formattedValue, bestObjective);
    fprintf("  RMS slip-gap error = %.3f deg; saturation = %.1f%%; wheel lift = %.1f%%\n", ...
        optimizationResult.bestRmsSlipGapError, ...
        100 .* optimizationResult.bestSaturationFraction, ...
        100 .* optimizationResult.bestWheelLiftFraction);
end

function metrics = evaluateBalanceParameterCandidates(candidateValues, baseParams, config, tire)
    candidateValues = reshape(candidateValues, 1, []);
    candidateCount = numel(candidateValues);
    metrics.objective = inf(size(candidateValues));
    metrics.rmsSlipGapError = inf(size(candidateValues));
    metrics.meanSlipGap = nan(size(candidateValues));
    metrics.saturationFraction = zeros(size(candidateValues));
    metrics.wheelLiftFraction = zeros(size(candidateValues));
    optimizationOperating.a_y_g = reshape(config.gValues, 1, []);
    optimizationOperating.a_x_g = zeros(size(optimizationOperating.a_y_g));

    for candidateIdx = 1:candidateCount
        squaredErrorSum = 0;
        slipGapSum = 0;
        validPointCount = 0;
        saturatedPointCount = 0;
        wheelLiftPointCount = 0;
        totalPointCount = 0;
        candidateParams = baseParams;
        candidateParams.(char(config.parameterName)) = candidateValues(candidateIdx);

        for radiusIdx = 1:numel(config.radii)
            candidateParams.r_corner = config.radii(radiusIdx);
            balance = evaluateVehicleBalance(candidateParams, optimizationOperating, tire);
            slipGap = balance.frontSA - balance.rearSA;
            saturated = balance.frontSA >= config.slipCapDeg ...
                | balance.rearSA >= config.slipCapDeg;
            wheelLift = min(balance.wheelLoads, [], 1) <= 0;
            valid = isfinite(slipGap) & ~saturated & ~wheelLift;

            squaredErrorSum = squaredErrorSum ...
                + sum((slipGap(valid) - config.targetSlipGapDeg).^2);
            slipGapSum = slipGapSum + sum(slipGap(valid));
            validPointCount = validPointCount + nnz(valid);
            saturatedPointCount = saturatedPointCount + nnz(saturated);
            wheelLiftPointCount = wheelLiftPointCount + nnz(wheelLift);
            totalPointCount = totalPointCount + numel(slipGap);
        end

        if validPointCount > 0
            metrics.rmsSlipGapError(candidateIdx) = ...
                sqrt(squaredErrorSum ./ validPointCount);
            metrics.meanSlipGap(candidateIdx) = slipGapSum ./ validPointCount;
        end
        metrics.saturationFraction(candidateIdx) = ...
            saturatedPointCount ./ totalPointCount;
        metrics.wheelLiftFraction(candidateIdx) = ...
            wheelLiftPointCount ./ totalPointCount;
        metrics.objective(candidateIdx) = metrics.rmsSlipGapError(candidateIdx) ...
            + config.saturationPenalty .* metrics.saturationFraction(candidateIdx) ...
            + config.wheelLiftPenalty .* metrics.wheelLiftFraction(candidateIdx);
    end
end

function layout = plotBalanceParameterOptimization(parent, result, config, caseLabel)
    layout = tiledlayout(parent, 2, 1, "TileSpacing", "compact", "Padding", "compact");
    title(layout, config.figureTitle + ": " + caseLabel);
    displayedCoarseValues = config.displayScale .* result.coarseValues;
    displayedFineValues = config.displayScale .* result.fineValues;
    displayedOptimum = config.displayScale .* result.optimalValue;
    optimumLabel = "Optimum " + string(sprintf(char(config.valueFormat), displayedOptimum));

    objectiveAxes = nexttile(layout);
    plot(objectiveAxes, displayedCoarseValues, ...
        result.coarseMetrics.objective, "o-", "DisplayName", "Coarse search");
    hold(objectiveAxes, "on");
    plot(objectiveAxes, displayedFineValues, ...
        result.fineMetrics.objective, ".-", "LineWidth", 1.5, ...
        "MarkerSize", 14, "DisplayName", "Fine search");
    xline(objectiveAxes, displayedOptimum, "--", optimumLabel, ...
        "LabelOrientation", "horizontal", "HandleVisibility", "off");
    grid(objectiveAxes, "on");
    xlabel(objectiveAxes, config.xLabel);
    ylabel(objectiveAxes, "Penalized objective [deg-equivalent]");
    legend(objectiveAxes, "Location", "best");
    hold(objectiveAxes, "off");

    metricAxes = nexttile(layout);
    yyaxis(metricAxes, "left");
    rmsLine = plot(metricAxes, displayedFineValues, ...
        result.fineMetrics.rmsSlipGapError, "o-", ...
        "DisplayName", "RMS balance error [deg]");
    hold(metricAxes, "on");
    ylabel(metricAxes, "RMS balance error [deg]");
    yyaxis(metricAxes, "right");
    saturationLine = plot(metricAxes, displayedFineValues, ...
        100 .* result.fineMetrics.saturationFraction, "o-", ...
        "Color", [0.85, 0.33, 0.10], ...
        "DisplayName", "Slip-cap points [%]");
    wheelLiftLine = plot(metricAxes, displayedFineValues, ...
        100 .* result.fineMetrics.wheelLiftFraction, "s--", ...
        "Color", [0.49, 0.18, 0.56], ...
        "DisplayName", "Wheel-lift points [%]");
    ylabel(metricAxes, "Limited operating points [%]");
    xline(metricAxes, displayedOptimum, "--", ...
        "HandleVisibility", "off");
    grid(metricAxes, "on");
    xlabel(metricAxes, config.xLabel);
    legend(metricAxes, [rmsLine, saturationLine, wheelLiftLine], "Location", "best");
    subtitle(metricAxes, compose( ...
        "Target gap = +%.2f deg (slight understeer); radii = %s m; lateral acceleration = %s g", ...
        config.targetSlipGapDeg, mat2str(config.radii), mat2str(config.gValues)));
    hold(metricAxes, "off");
end

function mapResult = calculateCornerRadiusMap(baseParams, heatMap, tire)
    mapOperating.a_y_g = reshape(heatMap.a_y_g, 1, []);
    mapOperating.a_x_g = zeros(size(mapOperating.a_y_g));
    radiusCount = numel(heatMap.radiusValues);
    pointCount = numel(mapOperating.a_y_g);
    mapResult.g = mapOperating.a_y_g;
    mapResult.slipGap = nan(radiusCount, pointCount);
    mapResult.saturated = false(radiusCount, pointCount);
    mapResult.minInsideWheelLoad = nan(radiusCount, pointCount);

    for radiusIdx = 1:radiusCount
        radiusParams = baseParams;
        radiusParams.r_corner = heatMap.radiusValues(radiusIdx);
        radiusResult = evaluateVehicleBalance(radiusParams, mapOperating, tire);
        mapResult.slipGap(radiusIdx, :) = radiusResult.frontSA - radiusResult.rearSA;
        mapResult.saturated(radiusIdx, :) = ...
            radiusResult.frontSA >= heatMap.slipCapDeg ...
            | radiusResult.rearSA >= heatMap.slipCapDeg;
        mapResult.minInsideWheelLoad(radiusIdx, :) = min( ...
            radiusResult.wheelLoads([2, 4], :), [], 1);
    end

    mapResult.displayGap = mapResult.slipGap;
    mapResult.displayGap(mapResult.saturated) = NaN;
end

function plotCornerRadiusBalanceMap(ax, mapResult, heatMap, caseLabel)
    finiteGap = mapResult.displayGap(isfinite(mapResult.displayGap));
    if isempty(finiteGap)
        colorLimit = 1;
    else
        colorLimit = max(abs(finiteGap));
        if colorLimit == 0, colorLimit = 1; end
    end

    [gGrid, radiusGrid] = meshgrid(mapResult.g, heatMap.radiusValues);
    balanceImage = surface(ax, gGrid, radiusGrid, zeros(size(gGrid)), ...
        mapResult.displayGap, "EdgeColor", "none", "FaceColor", "texturemap");
    set(balanceImage, "AlphaData", double(isfinite(mapResult.displayGap)), ...
        "FaceAlpha", "texturemap", "AlphaDataMapping", "none");
    view(ax, 2);
    set(ax, "YDir", "normal", "Color", [0.72, 0.72, 0.72]);
    colormap(ax, blueWhiteRedMap(256));
    clim(ax, [-colorLimit, colorLimit]);
    hold(ax, "on");

    speedMph = sqrt(radiusGrid .* gGrid .* 9.81) .* 2.23694;
    [speedContour, speedHandle] = contour(ax, mapResult.g, heatMap.radiusValues, ...
        speedMph, heatMap.speedContoursMph, "k--", "LineWidth", 0.8);
    clabel(speedContour, speedHandle, "Color", "k", "FontWeight", "bold");

    if any(mapResult.saturated(:))
        markerColumns = unique(round(linspace(1, numel(mapResult.g), ...
            min(heatMap.saturationMarkerColumnCount, numel(mapResult.g)))));
        markerMask = false(size(mapResult.saturated));
        markerMask(:, markerColumns) = mapResult.saturated(:, markerColumns);
        scatter(ax, gGrid(markerMask), radiusGrid(markerMask), ...
            25, "s", "filled", "MarkerFaceColor", [0.15, 0.15, 0.15], ...
            "MarkerEdgeColor", "none", "MarkerFaceAlpha", 0.38);
    end

    contour(ax, mapResult.g, heatMap.radiusValues, mapResult.displayGap, ...
        [-heatMap.neutralBandDeg, heatMap.neutralBandDeg], ...
        "k:", "LineWidth", 1.3);
    addBalanceArrows(ax, mapResult, heatMap);

    xlabel(ax, "Lateral acceleration [g]");
    ylabel(ax, "Corner radius [m]");
    title(ax, "Corner-Radius Balance Map: " + caseLabel);
    subtitle(ax, ["Color = front - rear slip angle; right arrow = understeer; left arrow = oversteer", ...
                  "Black dashed = speed [mph]; dotted = near neutral; gray squares = tire-model slip cap"]);
    colorbarHandle = colorbar(ax);
    colorbarHandle.Label.String = "Front - rear slip angle [deg]";
    grid(ax, "off");
    hold(ax, "off");
end

function addBalanceArrows(ax, mapResult, heatMap)
    columnIdx = unique(round(linspace(1, numel(mapResult.g), ...
        min(heatMap.arrowColumnCount, numel(mapResult.g)))));
    arrowGap = mapResult.displayGap(:, columnIdx);
    finiteGap = arrowGap(isfinite(arrowGap));
    if isempty(finiteGap), return; end

    arrowScale = max(abs(finiteGap));
    [arrowG, arrowRadius] = meshgrid(mapResult.g(columnIdx), heatMap.radiusValues);
    arrowMask = isfinite(arrowGap) & abs(arrowGap) >= heatMap.neutralBandDeg;
    arrowGap = arrowGap(arrowMask);
    arrowG = arrowG(arrowMask);
    arrowRadius = arrowRadius(arrowMask);

    for arrowIdx = 1:numel(arrowGap)
        if arrowGap(arrowIdx) > 0
            arrowText = "\rightarrow";
        else
            arrowText = "\leftarrow";
        end
        arrowSize = 11 + 5 .* min(abs(arrowGap(arrowIdx)) ./ arrowScale, 1);
        text(ax, arrowG(arrowIdx), arrowRadius(arrowIdx), arrowText, ...
            "Color", "k", "FontSize", arrowSize, "FontWeight", "bold", ...
            "HorizontalAlignment", "center", "VerticalAlignment", "middle");
    end
end

function outputFile = caseOutputFile(baseOutputFile, caseInfo, caseIdx)
    if isscalar(caseInfo.labels)
        outputFile = baseOutputFile;
        return
    end
    [~, stem, extension] = fileparts(baseOutputFile);
    safeLabel = regexprep(caseInfo.labels(caseIdx), "[^A-Za-z0-9._-]", "_");
    outputFile = stem + "_" + safeLabel + extension;
end

function exportCornerRadiusMap(ax, fig, outputPath)
    try
        exportgraphics(ax, outputPath, "Resolution", 200);
    catch axesExportError
        if ~isgraphics(fig, "figure")
            warning("VehicleBalance:HeatMapExport", ...
                "Could not export %s because its figure was closed: %s", ...
                outputPath, axesExportError.message);
            return
        end
        try
            exportgraphics(fig, outputPath, "Resolution", 200);
        catch figureExportError
            warning("VehicleBalance:HeatMapExport", ...
                "Could not export %s. Axes export: %s Figure export: %s", ...
                outputPath, axesExportError.message, figureExportError.message);
        end
    end
end

function colorMap = blueWhiteRedMap(colorCount)
    halfCount = floor(colorCount ./ 2);
    lower = [linspace(0.10, 1, halfCount)', ...
             linspace(0.30, 1, halfCount)', ...
             ones(halfCount, 1)];
    upperCount = colorCount - halfCount;
    upper = [ones(upperCount, 1), ...
             linspace(1, 0.20, upperCount)', ...
             linspace(1, 0.10, upperCount)'];
    colorMap = [lower; upper];
end

function [cases, info] = expandSetupCases(setup)
    names = fieldnames(setup);
    isSweep = false(size(names));
    for idx = 1:numel(names)
        value = setup.(names{idx});
        validateattributes(value, {'numeric'}, ...
            {'real', 'finite', 'nonempty', 'vector'}, mfilename, "setup." + string(names{idx}));
        isSweep(idx) = numel(value) > 1;
    end
    sweepNames = names(isSweep);
    if isempty(sweepNames)
        cases = setup;
        info.labels = "Baseline";
        info.sweptFields = strings(0, 1);
        return
    end
    values = cellfun(@(name) reshape(setup.(name), 1, []), sweepNames, "UniformOutput", false);
    grids = cell(size(values));
    [grids{:}] = ndgrid(values{:});
    cases = repmat(setup, numel(grids{1}), 1);
    info.labels = strings(numel(cases), 1);
    for caseIdx = 1:numel(cases)
        parts = strings(1, numel(sweepNames));
        for sweepIdx = 1:numel(sweepNames)
            name = sweepNames{sweepIdx};
            cases(caseIdx).(name) = grids{sweepIdx}(caseIdx);
            parts(sweepIdx) = string(name) + "=" + compose("%.5g", grids{sweepIdx}(caseIdx));
        end
        info.labels(caseIdx) = strjoin(parts, ", ");
    end
    info.sweptFields = string(sweepNames);
end

function result = evaluateVehicleBalance(p, operating, tire)
    ayg = reshape(operating.a_y_g, 1, []);
    axg = reshape(operating.a_x_g, 1, []);
    if isscalar(axg), axg = repmat(axg, size(ayg)); end
    assert(numel(axg) == numel(ayg), ...
        "operating.a_x_g must be scalar or match operating.a_y_g.");
    ay = ayg .* p.g_ftps2;
    ax = axg .* p.g_ftps2;
    n = numel(ay);

    mt = p.W_tot ./ p.g_ftps2;
    muf = p.W_unsprungF ./ p.g_ftps2;
    mur = p.W_unsprungR ./ p.g_ftps2;
    ms = mt - muf - mur;
    cgz = (p.sprung_z .* ms + p.unsprung_z .* (muf + mur)) ./ mt;

    deltaO = atand(p.wheelbase ./ (p.r_corner ./ p.metersPerInch + p.TF ./ 2));
    deltaI = -deltaO .* (1 + p.ackermannCorrection .* deltaO) + 2 .* p.toeF;
    deltaIAck = -atand(p.wheelbase ./ (p.r_corner ./ p.metersPerInch - p.TF ./ 2));
    toeEff = (deltaI - deltaIAck) ./ 2;

    velocity = sqrt(p.r_corner .* ayg .* p.g_mps2);
    q = 0.5 .* p.rhoAir .* velocity.^2;
    downforce = q .* p.CL .* p.aeroArea .* p.newtonsToLbf;
    drag = q .* p.CD .* p.aeroArea .* p.newtonsToLbf;

    kfWheel = p.kWheel_f .* p.TF.^2 .* tand(1) ./ 2 .* p.rollStiffnessConversion;
    krWheel = p.kWheel_r .* p.TR.^2 .* tand(1) ./ 2 .* p.rollStiffnessConversion;
    kf = kfWheel + p.kRoll_f_arb;
    kr = krWheel + p.kRoll_r_arb;
    rollCouple = ay .* ms .* (p.sprung_z - (p.rc_zr + p.rc_zf) ./ 2);
    bodyRoll = rollCouple .* p.rollStiffnessConversion ./ (kf + kr);
    heave = downforce ./ (p.kWheel_f + p.kWheel_r);

    camFO = p.camberF - p.castor .* sind(deltaO) + p.KPI .* (1 - cosd(deltaO));
    camFI = p.camberF - p.castor .* sind(deltaI) + p.KPI .* (1 - cosd(deltaI));
    camFO = camFO + p.frontHeaveCamberGain .* heave + p.frontRollCamberGain .* bodyRoll;
    camFI = camFI + p.frontHeaveCamberGain .* heave - p.frontRollCamberGain .* bodyRoll;
    camRO = p.camberR + p.rearHeaveCamberGain .* heave + p.rearRollCamberGain .* bodyRoll;
    camRI = p.camberR + p.rearHeaveCamberGain .* heave - p.rearRollCamberGain .* bodyRoll;

    staticRR = (p.W_tot .* (1-p.weightDistF) + downforce .* (1-p.DFDistF)) .* (1-p.weightDistL);
    staticRL = (p.W_tot .* (1-p.weightDistF) + downforce .* (1-p.DFDistF)) .* p.weightDistL;
    staticFR = (p.W_tot .* p.weightDistF + downforce .* p.DFDistF) .* (1-p.weightDistL);
    staticFL = (p.W_tot .* p.weightDistF + downforce .* p.DFDistF) .* p.weightDistL;
    uf = ay .* muf .* p.r_l ./ p.TF;
    ur = ay .* mur .* p.r_l ./ p.TR;
    linkF = ay .* ms .* p.weightDistF .* p.rc_zf ./ p.TF;
    linkR = ay .* ms .* (1-p.weightDistF) .* p.rc_zr ./ p.TR;
    rollF = kf ./ (kr+kf) .* rollCouple ./ p.TF;
    rollR = kr ./ (kr+kf) .* rollCouple ./ p.TR;
    jack = p.jackingLoadAt20Deg ./ 20 .* deltaO ./ 2;
    longitudinal = ax .* mt .* cgz ./ p.wheelbase;

    loads = zeros(4,n); % front-outside, front-inside, rear-outside, rear-inside
    loads(1,:) = staticFL + rollF + linkF + uf - jack - longitudinal./2;
    loads(2,:) = staticFR - rollF - linkF - uf + jack - longitudinal./2;
    loads(3,:) = staticRL + rollR + linkR + ur + jack + longitudinal./2;
    loads(4,:) = staticRR - rollR - linkR - ur - jack + longitudinal./2;
    frontLoad = loads(1,:) + loads(2,:);
    rearLoad = loads(3,:) + loads(4,:);
    totalLoad = frontLoad + rearLoad;
    leftLoad = loads(1,:) + loads(3,:);
    rightLoad = totalLoad - leftLoad;

    fyTotal = mt .* ay;
    fyF = fyTotal .* p.weightDistF;
    fyR = fyTotal .* (1-p.weightDistF);
    [saF,saR] = solveSlip(loads,frontLoad,rearLoad,fyF,fyR,camFO,camFI,camRO,camRI,toeEff,p.toeR,tire,true);
    parasitic = ay.*mt.*sin(saR) + 0.5.*ay.*mt.*sin(saF-saR) + p.parasiticDragCoeff.*p.W_tot;
    demandX = ax.*mt + parasitic + drag;
    braking = ax < 0;
    fxF = zeros(size(ax)); fxR = demandX;
    fxF(braking) = demandX(braking).*p.brakeBiasF;
    fxR(braking) = demandX(braking).*(1-p.brakeBiasF);
    fyF = hypot(fyF,fxF); fyR = hypot(fyR,fxR);
    deltaRolling = p.rollingResistanceCoeff.*(leftLoad-rightLoad).*p.TF./2./p.wheelbase;

    [saF,saR] = solveSlip(loads,frontLoad,rearLoad,fyF,fyR,camFO,camFI,camRO,camRI,toeEff,p.toeR,tire,true);
    [mzFO,mzFI,mzRO,mzRI] = aligningMoments(loads,camFO,camFI,camRO,camRI,saF,saR,tire);
    deltaAligning = (mzFI+mzFO+mzRO+mzRI)./p.wheelbase;
    deltaInduced = (loads(1,:)-loads(2,:)).*ay./p.g_ftps2.*sin(saF-saR).*p.TF./2./p.TF;
    fyF = fyF + deltaRolling + deltaAligning + deltaInduced;
    fyR = fyR - deltaRolling - deltaAligning - deltaInduced;
    [saF,saR] = solveSlip(loads,frontLoad,rearLoad,fyF,fyR,camFO,camFI,camRO,camRI,toeEff,p.toeR,tire,false);

    result.params=p; result.g=ayg; result.velocity=velocity;
    result.frontSA=saF; result.rearSA=saR;
    result.FyFront=fyF; result.FyRear=fyR; result.FxFront=fxF; result.FxRear=fxR;
    result.wheelLoads=loads; result.frontLoad=frontLoad; result.rearLoad=rearLoad;
    result.camber=[camFO;camFI;camRO;camRI];
    result.MZFrontOutside=mzFO; result.MZFrontInside=mzFI;
    result.roadWheelSteer=(abs(deltaO)+abs(deltaI))./2;
    result.steeringAngle=saF-saR+result.roadWheelSteer;
end

function [saF,saR] = solveSlip(loads,fLoad,rLoad,fyF,fyR,cFO,cFI,cRO,cRI,toeF,toeR,tire,radians)
    n=numel(fyF); saF=zeros(1,n); saR=zeros(1,n); scale=1;
    if radians, scale=pi/180; end
    for i=1:n
        saF(i)=findSlip(loads(1,i),fLoad(i),fyF(i),cFO(i),cFI(i),toeF,tire.P,tire.L).*scale;
        saR(i)=findSlip(loads(3,i),rLoad(i),fyR(i),cRO(i),cRI(i),toeR,tire.P,tire.L).*scale;
    end
end

function [fo,fi,ro,ri] = aligningMoments(loads,cFO,cFI,cRO,cRI,saF,saR,tire)
    n=numel(saF); fo=zeros(1,n); fi=fo; ro=fo; ri=fo;
    for i=1:n
        fo(i)=pacejkaMZ(tire.PM,tire.LMZ,loads(1,i),deg2rad(cFO(i)),saF(i)).*12;
        fi(i)=pacejkaMZ(tire.PM,tire.LMZ,loads(2,i),-deg2rad(cFI(i)),saF(i)).*12;
        ro(i)=pacejkaMZ(tire.PM,tire.LMZ,loads(3,i),deg2rad(cRO(i)),saR(i)).*12;
        ri(i)=pacejkaMZ(tire.PM,tire.LMZ,loads(4,i),-deg2rad(cRI(i)),saR(i)).*12;
    end
end

function out = evaluateSteeringEffort(balance,setups)
    out.columnTorque=zeros(numel(setups),numel(balance.g)); out.wheelForce=out.columnTorque;
    for i=1:numel(setups)
        s=setups(i); k=deg2rad(s.KPI); steer=deg2rad(s.steerAngle); caster=deg2rad(s.castor);
        mv=-balance.frontLoad.*s.scrubRadius.*sin(k).*sin(steer) ...
           +(balance.wheelLoads(1,:)-balance.wheelLoads(2,:)).*s.scrubRadius.*sin(caster).*sin(steer);
        ml=-balance.FyFront.*s.trail;
        ma=-(balance.MZFrontInside+balance.MZFrontOutside).*cos(hypot(k,caster));
        rack=(mv+ml+ma)./(s.steeringArm.*cos(steer));
        out.columnTorque(i,:)=rack.*s.pinionRadius;
        out.wheelForce(i,:)=-out.columnTorque(i,:)./s.steeringWheelDiameter;
    end
end

function plotVehicleBalance(results,steeringResults,info,steeringSetups)
    colors=lines(max(2*numel(results),3)); labels=strings(0);
    assumptionText=constantRadiusAssumptionText(results);
    figure("Name","Front vs Rear Slip Angle","NumberTitle","off"); hold on; grid on;
    for c=1:numel(results)
        plot(results(c).g,results(c).frontSA,"Color",colors(2*c-1,:));
        plot(results(c).g,results(c).rearSA,"--","Color",colors(2*c,:));
        labels(end+1:end+2)=[info.labels(c)+" - Front",info.labels(c)+" - Rear"];
    end
    xlabel("Cornering g-force"); ylabel("SA [deg]"); title("Comparison of Front vs Rear Slip Angle");
    subtitle(assumptionText); legend(labels);

    plotAxles(results,info,colors,"Required Lateral Grip","Required lateral grip [lbf]",assumptionText, ...
        @(r)r.FyFront,@(r)r.FyRear,labels);
    plotAxles(results,info,colors,"Front vs Rear Load Transfer","Load transfer [lbf]",assumptionText, ...
        @(r)r.wheelLoads(1,:)-r.wheelLoads(2,:),@(r)r.wheelLoads(3,:)-r.wheelLoads(4,:),labels);
    plotAxles(results,info,colors,"Inside Wheel Loads","Inside load [lbf]",assumptionText, ...
        @(r)r.wheelLoads(2,:),@(r)r.wheelLoads(4,:),labels);

    figure("Name","Understeer Gradient","NumberTitle","off"); hold on; grid on;
    for c=1:numel(results), plot(results(c).g,results(c).steeringAngle); end
    xlabel("Lateral acceleration [g]"); ylabel("Steering angle [deg]"); title("Understeer Gradient");
    subtitle(assumptionText); legend(info.labels);

    steeringLabels=strings(0);
    figure("Name","Steering Wheel Force Comparison","NumberTitle","off"); hold on; grid on;
    steeringLineCount=numel(results)*numel(steeringSetups);
    wheelForceLines=gobjects(steeringLineCount,1);
    steeringLineIdx=0;
    for c=1:numel(results)
        for s=1:numel(steeringSetups)
            steeringLineIdx=steeringLineIdx+1;
            wheelForceLines(steeringLineIdx)=plot( ...
                results(c).g,steeringResults{c}.wheelForce(s,:));
            steeringLabels(end+1)=info.labels(c)+" - "+string(steeringSetups(s).label); %#ok<AGROW>
        end
    end
    xlim([0.8,1.9]); xlabel("Cornering G-force"); ylabel("Steering Force [lbf]");
    title("Steering Force at 10 deg Steering Angle, 8.5 in Wheel");
    subtitle(assumptionText); legend(wheelForceLines,steeringLabels);

    figure("Name","Steering Column Torque Comparison","NumberTitle","off"); hold on; grid on;
    columnTorqueLines=gobjects(steeringLineCount,1);
    steeringLineIdx=0;
    for c=1:numel(results)
        for s=1:numel(steeringSetups)
            steeringLineIdx=steeringLineIdx+1;
            columnTorqueLines(steeringLineIdx)=plot( ...
                results(c).g,-steeringResults{c}.columnTorque(s,:));
        end
    end
    xlim([0.8,1.9]); xlabel("Cornering G-force"); ylabel("Column Torque [lbf-in]");
    title("Steering Column Torque at 10 deg Steering Angle");
    subtitle(assumptionText); legend(columnTorqueLines,steeringLabels);
end

function plotAxles(results,~,colors,figureName,yText,assumptionText,frontFn,rearFn,labels)
    figure("Name",figureName,"NumberTitle","off"); hold on; grid on;
    for c=1:numel(results)
        plot(results(c).g,frontFn(results(c)),"Color",colors(2*c-1,:));
        plot(results(c).g,rearFn(results(c)),"--","Color",colors(2*c,:));
    end
    xlabel("Cornering g-force"); ylabel(yText); title(figureName);
    subtitle(assumptionText); legend(labels);
end

function textLines = constantRadiusAssumptionText(results)
    radii = arrayfun(@(result) result.params.r_corner, results);
    uniqueRadii = unique(radii, "stable");
    gMin = min(arrayfun(@(result) min(result.g), results));
    gMax = max(arrayfun(@(result) max(result.g), results));
    speedMinMph = min(arrayfun(@(result) min(result.velocity), results)) .* 2.23694;
    speedMaxMph = max(arrayfun(@(result) max(result.velocity), results)) .* 2.23694;

    if isscalar(uniqueRadii)
        radiusText = compose("Constant corner radius = %.4g m; lateral acceleration = %.3g-%.3g g.", ...
            uniqueRadii, gMin, gMax);
    else
        radiusList = strjoin(compose("%.4g", uniqueRadii), ", ");
        radiusText = "Each case holds corner radius constant at " + radiusList ...
            + " m; lateral acceleration = " + compose("%.3g-%.3g g.", gMin, gMax);
    end
    speedText = compose("Speed is implied by v = sqrt(r*a_y): %.1f-%.1f mph; speed is not held constant.", ...
        speedMinMph, speedMaxMph);
    textLines = [radiusText, speedText];
end
