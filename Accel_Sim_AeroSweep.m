clear all; clc; close all;
tmax=8.901;
tmin=4.169



%% 1. POWERTRAIN PARAMETERS
T = [29.8, 36.6, 44.7, 50.2, 52.9, 48.8, 40.7, 36.6, 29.8, 23]; % Torque [N*m]
N = [5, 6, 7, 8, 9, 10, 11, 12, 13, 14] * 1000; % RPM
r = [2.75, 2, 1.667, 1.444, 1.304, 1.208]; % Gear ratios 1-6
r_p = 2.111; % Primary gear reduction
r_f = 32/11; % Final drive ratio
t_shift = 0.1; % Shift time [s]

%% 2. VEHICLE SETUP
h = 0.3175; m = 272; TL = 1.5367; 
weightDistF = 0.485; b = TL*(1-weightDistF);
R_tire = 0.2032; R_loaded = 0.193;
mu = 1.5; g = 9.81; rho = 1.189;
aeroBalance_R = 0.56; % Rear aero balance
F_rr = m*g*0.02; % Rolling resistance
SR = 0.1; % Slip ratio

%% 3. SWEEP CONFIGURATION
% Define the ranges you want to test here
CDA_vec = [2,2.1]; % Drag coefficient sweep
CLA_vec = [4,4.1]; % Downforce coefficient sweep

% Preallocate results matrix
results_time = zeros(length(CDA_vec), length(CLA_vec));

%% 4. SIMULATION CONSTANTS & FUNCTIONS
rpm_func = @(V, gear_idx) (SR*V+V)/R_tire*(r(gear_idx)*r_p*r_f)*60/(2*pi); 
D = 75; % Drag strip length [m]
dt = 0.001; % Time step

[T_max, i_Tmax] = max(T);
rpm_Tmax = N(i_Tmax);
% Ideal speed for clutch release (starting gear = 1)
v_release = rpm_Tmax/(r(1)*r_p*r_f)*2*pi/60*R_tire/(1+SR);

%% 5. THE SWEEP ENGINE
fprintf('Starting Sweep Simulation...\n');

for c = 1:length(CDA_vec)
    for l = 1:length(CLA_vec)
        
        % Current aero values for this iteration
        curr_CDA = CDA_vec(c);
        curr_CLA = CLA_vec(l);
        
        % Initialize simulation variables
        x_i = 0; v_i = 0.001; t_i = 0; gear = 1;
        
        while x_i < D
            % Aero forces
            F_drag = 0.5 * rho * curr_CDA * v_i^2;
            F_downforce = 0.5 * rho * curr_CLA * v_i^2;
            
            % Traction limit
            F_max = (mu*m*g*b/TL + mu*F_downforce*aeroBalance_R) / (1 - h/TL*mu);
            
            % Engine Logic
            if v_i < v_release
                rpm_i = rpm_Tmax;
            else
                rpm_i = rpm_func(v_i, gear);
                % Check for upshift
                if gear < length(r)
                    rpm_next = rpm_func(v_i, gear+1);
                    % Ensure rpm is within torque curve bounds for comparison
                    if rpm_i > max(N), rpm_i = max(N); end
                    
                    p_curr = rpm_i * interp1(N, T, rpm_i, 'linear', 'extrap');
                    p_next = rpm_next * interp1(N, T, rpm_next, 'linear', 'extrap');
                    
                    if p_curr < p_next
                        gear = gear + 1;
                        t_i = t_i + t_shift;
                        x_i = x_i + v_i * t_shift;
                        rpm_i = rpm_func(v_i, gear);
                    end
                end
            end
            
            % Resolve force and acceleration
            % Bound RPM to torque curve data
            rpm_lookup = max(min(rpm_i, max(N)), min(N));
            T_i = interp1(N, T, rpm_lookup);
            F_i = T_i * (r(gear) * r_p * r_f) / R_loaded;
            
            if F_i > F_max, F_i = F_max; end
            
            F_accel = F_i - F_drag - F_rr;
            a = F_accel / (1.05 * m);
            
            % Update kinematic state
            v_i = v_i + a * dt;
            x_i = x_i + v_i * dt;
            t_i = t_i + dt;
            
            % Safety break for stalled simulations
            if t_i > 20, break; end 
        end
        
        results_time(c, l) = t_i;
    end
end

fprintf('Simulation Complete.\n');

%% 6. VISUALIZATION
score_vec=95.5.*((tmax./results_time)-1)/((tmax/tmin)-1)+4.5;

figure('Color', 'w', 'Name', 'Aero Sensitivity Analysis');
surf(CLA_vec, CDA_vec, score_vec);
shading interp; % Smooths the colors
colorbar;
xlabel('CLA (Downforce)'); 
ylabel('CDA (Drag)'); 
zlabel('score');
title('Effect of Aero Coefficients on 75m Sprint Time');
view(45, 30); % Rotates to a nice 3D perspective