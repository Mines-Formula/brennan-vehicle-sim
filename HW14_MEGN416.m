
%% 4.47
close all;   
clear;        
clc; 

k_h = 10000; %[N/m]
k_s = 20000; %[N/m]
J = 75; %[kg]
m_2 = 100; %[kg]
m_3 = 3000; %[kg]

x0 = [0;0;0]; %initial position
xdot0 = [0;0;1]; %initial velocity

M = diag([J,m_2,m_3]); %mass matrix
K = [k_h,-k_h,0;-k_h,k_h+k_s,-k_s;0,-k_s,k_s]; %stiffness matrix
Kt = M^(-1/2)*K*M^(-1/2); %transformed stiffness matrix
% Calculate the eigenvalues and eigenvectors of the transformed stiffness matrix
[P, L] = eig(Kt);
W = sqrt(L); %natural frequencies matrix
w = diag(W); %natrual frequencies vector

%define S, r0, and rdot0
S = M^(-1/2)*P;
r0 = S^-1*x0;
rdot0 = S^-1*xdot0;

%define symbolic variables
syms t

w_sym = sym(w);
r0_sym = sym(r0);
rdot0_sym = sym(rdot0);
r_sym = sym(zeros(3,1));

tol = 1e-6; % tolerance for "zero" frequency (rigid body mode)

%built r(t) equations
for i = 1:length(w_sym)
    if abs(w_sym(i)) < tol
        % rigid body mode: r_i(t) = r0_i + rdot0_i * t
        r_sym(i) = r0_sym(i) + rdot0_sym(i)*t;
    else
        % vibrating mode: usual solution
        r_sym(i) = r0_sym(i)*cos(w_sym(i)*t) + ...
                   (rdot0_sym(i)/w_sym(i))*sin(w_sym(i)*t);
    end
end

S_sym = sym(S);
x_sym = S_sym*r_sym; %convert from mode to engineering space
x_sym_decimal = vpa(x_sym, 4) %output in decimal format

%% 4.49
close all;   
clear;        
clc; 

%define symbolic variables
syms m;
syms E;
syms I;
syms L;

%define M and K matrices
M = sym(diag([1,4,1])*m);
K = sym([3,-3,0;-3,6,-3;0,-3,3]*E*I/L^3);


Kt = sym(M^(-1/2)*K*M^(-1/2)); %transformed stiffness matrix
[P,L] = eig(Kt); %solve eigen values/vectors

P = double(P); %convert P to double (numeric)
P = P./vecnorm(P); %normalize P columns
P = sym(P); %convert back to symbolic
U = M^(-1/2)*P; %solve for mode shapes
U_decimal = vpa(U,4) %output mode shapes
W = vpa(diag(sqrt(L)),4) %output natural frequencies

%% 4.51
close all;   
clear;        
clc; 

%define mass and stiffness matrix
M = diag([10,10,10,10]);
K = [500,0,-100,0;...
    0,400,0,0;...
    -100,0,500,0;...
    0,0,0,600];

Kt = M^(-1/2)*K*M^(-1/2); %transformed stiffness matrix
%solve eignenvalues/vectors
[P,L] = eig(Kt);
P./vecnorm(P) %output normalized eigenvectors
diag(L) %output eigenvalues

%% 4.53/4.54
close all;   
clear;        
clc; 

%define symbolic variables
syms m;
syms E;
syms I;
syms L;

%define M and K matrices
M = sym(diag([1,1,1])*m);
K = sym([9/64,1/6,13/192;1/6,1/3,1/6;13/192,1/6,9/64]*E*I/L^3);


Kt = sym(M^(-1/2)*K*M^(-1/2)); %transformed stiffness matrix
[P,L] = eig(Kt); %solve eigen values/vectors

P = double(P); %convert P to double (numeric)
P = P./vecnorm(P); %normalize P columns
P = sym(P); %convert back to symbolic
U = M^(-1/2)*P; %solve for mode shapes
U_decimal = vpa(U*m^(1/2),4) %output mode shapes
W = vpa(diag(sqrt(L)),4) %output natural frequencies

%% 4.56
close all;   
clear;        
clc; 

k_h = 10000; %[N/m]
k_s = 20000; %[N/m]
J = 75; %[kg]
m_2 = 100; %[kg]
m_3 = 2000; %[kg]

x0 = [0;0;0]; %initial position
xdot0 = [0;0;1]; %initial velocity

M = diag([J,m_2,m_3]); %mass matrix
K = [k_h,-k_h,0;-k_h,k_h+k_s,-k_s;0,-k_s,k_s]; %stiffness matrix
Kt = M^(-1/2)*K*M^(-1/2); %transformed stiffness matrix
% Calculate the eigenvalues and eigenvectors of the transformed stiffness matrix
[P, L] = eig(Kt);
W = sqrt(L); %natural frequencies matrix
w = diag(W); %natrual frequencies vector

%define S, r0, and rdot0
S = M^(-1/2)*P;
r0 = S^-1*x0;
rdot0 = S^-1*xdot0;

%define symbolic variables
syms t

w_sym = sym(w);
r0_sym = sym(r0);
rdot0_sym = sym(rdot0);
r_sym = sym(zeros(3,1));

tol = 1e-6; % tolerance for "zero" frequency (rigid body mode)

%built r(t) equations
for i = 1:length(w_sym)
    if abs(w_sym(i)) < tol
        % rigid body mode: r_i(t) = r0_i + rdot0_i * t
        r_sym(i) = r0_sym(i) + rdot0_sym(i)*t;
    else
        % vibrating mode: usual solution
        r_sym(i) = r0_sym(i)*cos(w_sym(i)*t) + ...
                   (rdot0_sym(i)/w_sym(i))*sin(w_sym(i)*t);
    end
end

S_sym = sym(S);
x_sym = S_sym*r_sym; %convert from mode to engineering space
x_sym_decimal = vpa(x_sym, 4) %output in decimal format
