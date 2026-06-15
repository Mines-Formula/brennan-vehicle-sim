mdl = 'ConfiguredVirtualVehicleModel';
open_system(mdl);

%%

%Test 1 
in(1) = Simulink.SimulationInput(mdl); 
in(1) = setParamforManeuverAndDriver('ConfiguredVirtualVehicleModel','Increasing Steer', 'Disabled', 'Predictive Driver',1, in(1), 'FSAE_IC',1);
simout = sim(in, 'ShowSimulationManager', 'on');
save('simout.mat','simout','-v7.3');
