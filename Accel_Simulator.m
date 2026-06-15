clear all
clc
close all

%powertrain parameters
T = [27.1,34.6,38.6,37.7,36.9,38.6,40.7,41.4,42.7,47.2,...
     46.8,46.1,46.4,45.8,43.1,40.0,37.3,33.9,30.5,27.1]*1.07; % Torque [N*m]
N = [3000,3553,4105,4658,5211,5763,6316,6868,7421,7974,...
     8526,9079,9632,10184,10737,11289,11842,12395,12947,13500]; % RPM
% Calculate the power using the formula P = T * N / 9550
P = T .* N / 9.55; % Power [W]
r = [2.75, 2, 1.667, 1.444, 1.304, 1.208]; %gear ratios 1-5
r_p = 2.111; %primary gear reduction
r_f = 32/11; %final drive ratio
t_shift = 0.1; %shift time [s]

%vehicle setup parameters
h = 0.3; %cg height [m]
TL = 1.5367; %track length [m]
m = 272; %mass with driver [kg]  
weightDistF = 0.485; %percent of static weight on front axle
R_tire = 0.2032; %nominal tire radius [m]
R_loaded = 0.193; %loaded tire radius [m]
b = TL*(1-weightDistF); %x distance from front axle to cg [m]
mu = 1.5-0.00065*(m-272); %approximate loaded longitidunal friction coefficient
g = 9.81;

%aero drag
CDA = 1.5; % CD*frontal area [m^2]
CLA = 3; % CL*frontal area [m^2]
aeroBalance_R = 0.58; %percent rear aerobalance
rho = 1.189; % air density [kg/m^3]
F_a_drag = @(v) 0.5*rho*CDA*v^2;
F_a_downforce = @(v) 0.5*rho*CLA*v^2;

%rolling resistance
F_rr = m*g*0.03;

SR = 0.1; %assumed slip ratio
%define rpm function. Inputs speed V and gear n, outputs engine RPM
rpm = @(V, n) (SR*V+V)/R_tire*(r(n)*r_p*r_f)*60/(2*pi); 

D = 75; %drag strip length [m]
rollout = 0.3; %rollout distance [m]
start = 0; %boolean that markes crossing start line 
gear = 1; %starting gear
x_i = 0; %initialize position
v_i = 0; %initialize velocity
t_i = 0; %initialize time vector
dt = 0.001; %time step
F_max_i = (mu*m*g*b/TL)/(1-h/TL*mu) %traction limit
i = 0;

[T_max, i_Tmax] = max(T); %max torque value and index
rpm_Tmax = N(i_Tmax); %rpm which peak torque occurs
rpm_Tmax = 7000; %manually overide launch rpm
v_release = rpm_Tmax/(r(gear)*r_p*r_f)*2*pi/60*R_tire/(1+SR); %ideal speed for clutch release

%vectors to store and starting values:
X = [0,1,0,0,0,0,0,0,0] %t,gear, rpm, x, v, a, F, F_accel, F_max

while x_i < D + rollout
    F_max_i = (mu*m*g*b/TL + mu*F_a_downforce(v_i)*aeroBalance_R)/(1-h/TL*mu); %traction limit
    if x_i > rollout && start == 0
        t_rollout = t_i;
        i_rollout = i;
        start = 1;
    end
    if v_i < v_release
        rpm_i = rpm_Tmax;
    else
        rpm_i = rpm(v_i, gear);
        %check if next gear makes more power
        if gear < length(r)
        %if rpm_i > 13400 use to manually set shift point
        if rpm_i*interp1(N,T,rpm_i) < rpm(v_i, gear+1)*interp1(N,T,rpm(v_i, gear+1))
            gear = gear + 1 %shift to next gear
            t_i = t_i + t_shift; %add shift time
            x_i = x_i + v_i*t_shift; %assumes speed remaints roughly constant during shift
            rpm_i = rpm(v_i, gear); %update rpm for new gear
        end
        end
    end
    T_i = interp1(N,T,rpm_i);
    F_i = T_i*(r(gear)*r_p*r_f)/R_loaded;
    if F_i > F_max_i
        F_i = F_max_i;
    end
    F_accel_i = F_i - F_a_drag(v_i) - F_rr; %force contributing to acceleration
    a_i = F_accel_i / (1.05*m); %instantaneous acceleration (5% adjustment for rotating mass)
    v_i = v_i + a_i*dt; %update velocity
    x_i = x_i + v_i * dt; % update position
    t_i = t_i + dt; % update time vector
    i = i + 1; %increase index
    g_pull(i) = a_i/9.81;
    X(size(X,1)+1,:) = [t_i, gear,rpm_i, x_i, v_i, a_i/9.81, F_i, F_i-F_accel_i, F_max_i];
end

t_i-t_rollout
X = X(i_rollout:size(X,1),:); %remove rollout portion
X(:,1) = X(:,1)-X(1,1); %reset time at start line
X(:,4) = X(:,4)-X(1,4); %reset distance at start line
A = array2table(X,...
    "VariableNames", {'Time', 'Gear', 'RPM','Distance [m]','Velocity [m/s]'...
    'Acceleration [g]', 'Tractive Force [N]', 'Drag Forces [N]', 'Traction Limit [N]'});

%% Create  Traction Limit Plots
% Create Figure for Force Analysis
figure('Name', 'Traction and Tractive Force Analysis', 'NumberTitle', 'off');

% Top Plot: Forces vs Time
subplot(3,1,1);
plot(A.Time, A.("Tractive Force [N]"), 'b', 'LineWidth', 1.5); 
hold on;
plot(A.Time, A.("Traction Limit [N]"), 'r--', 'LineWidth', 1.5);
xlabel('Time [s]');
ylabel('Force [N]');
title('Forces vs Time');
legend('Tractive Force', 'Traction Limit', 'Location', 'best');
grid on;

% Middle Plot: Forces vs Distance
subplot(3,1,2);
plot(A.("Distance [m]"), A.("Tractive Force [N]"), 'b', 'LineWidth', 1.5); 
hold on;
plot(A.("Distance [m]"), A.("Traction Limit [N]"), 'r--', 'LineWidth', 1.5);
xlabel('Distance [m]');
ylabel('Force [N]');
title('Forces vs Distance');
legend('Tractive Force', 'Traction Limit', 'Location', 'best');
grid on;

% Bottom Plot: Forces vs Speed
subplot(3,1,3);
plot(A.("Velocity [m/s]"), A.("Tractive Force [N]"), 'b', 'LineWidth', 1.5); 
hold on;
plot(A.("Velocity [m/s]"), A.("Traction Limit [N]"), 'r--', 'LineWidth', 1.5);
xlabel('Velocity [m/s]');
ylabel('Force [N]');
title('Forces vs Speed');
legend('Tractive Force', 'Traction Limit', 'Location', 'best');
grid on;

% Adjust layout to prevent overlap
sgtitle('Accel Simulation: Tractive Force vs. Traction Limit');

%% Create Shift Points Plot
% Create Figure for RPM vs Velocity
figure('Name', 'Shift Point Analysis', 'NumberTitle', 'off');

% Plot the main RPM curve
plot(A.("Velocity [m/s]"), A.RPM, 'LineWidth', 2, 'Color', [0 0.4470 0.7410]);
hold on;

% Find shift points
shift_indices = find(diff(A.Gear) > 0);
v_shifts = A.("Velocity [m/s]")(shift_indices);
rpm_shifts = A.RPM(shift_indices);
gear_labels = A.Gear(shift_indices);

% Plot shift points: Just a clean red dot
hShift = plot(v_shifts, rpm_shifts, 'ro', ...
    'MarkerSize', 6, ...
    'MarkerFaceColor', 'r', ...
    'MarkerEdgeColor', 'r'); % Ensures the edge is the same color as the face

% Add data labels (Removed BackgroundColor to get rid of the square)
for j = 1:length(v_shifts)
    label_text = {sprintf('Shift %d \\rightarrow %d', gear_labels(j), gear_labels(j)+1), ...
                  sprintf('%.1f m/s', v_shifts(j)), ...
                  sprintf('%.0f RPM', rpm_shifts(j))};
    
    % Using 'Interpreter', 'tex' for the arrow and moving alignment to clear the dot
    text(v_shifts(j), rpm_shifts(j) + 150, label_text, ...
        'VerticalAlignment', 'bottom', ...
        'HorizontalAlignment', 'center', ...
        'FontSize', 8, ...
        'FontWeight', 'bold', ...
        'Interpreter', 'tex'); 
end

xlabel('Velocity [m/s]');
ylabel('Engine RPM');
title('Engine RPM vs. Velocity (Shift Point Detail)');
grid on;
legend(hShift, 'Shift Points', 'Location', 'southeast');

% Add a bit of padding to the top of the plot for labels
ylim([min(A.RPM)*0.8, max(A.RPM)*1.2]);