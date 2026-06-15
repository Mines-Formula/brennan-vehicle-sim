%Created by Payton Murphy-Blanchard

function MZ0 = pacejkaMZ(P, Q, L, FZ, IA, alpha, FY);

%P: The coefficients derived from the FY optimization.
%Q: Coefficients being optimized from the textbook
%L: Lambdas in the textbook
%IA: Inclination Angle
%alpha: Slip angle
%FY: Cornering Force input (currently not used in this build)

% inputs 24 Q coefficients,
% Estimated coefficients are as follows (found in Appen:
%Q = [0.007, -0.002, 0.147, 0.004, 8.964, -1.106, -0.842, 0, -0.227, 1.180, 0.1, -0.001, 0.007, 13.05, -1.609, -0.359, 0, 0.174, -0.896, 0, -0.008, 0, -0.296, -0.009];
%L = [1, 1, 1, 1, 1, 1, 1, 1];

%Placeholder vars for now:

Vcx  = 1;
Vc   = 1;
%Vcy  = 0;
Kya  = 1;
Kyao = 1;
Kyy0 = 1;

% This is Brennan's pacejka.m used to find FY
dfz = (FZ - P(1)) / P(1);
Svy = FZ .* (P(16) + P(17) * dfz + (P(18) + P(19) * dfz) .* IA) * L(8) * L(5);
Ey = (P(6) + P(7) * dfz) .* (1 - (P(8) + P(9) * IA) .* sign(alpha)) * L(7);
Shy = (P(13) + P(14) * dfz + P(15) * IA) * L(6);
alphaY = alpha + Shy; 
Cy = P(2) * L(2);
Dy = FZ .* (P(3) + P(4) * dfz) .* (1-P(5) * IA.^2) * L(1);
x1 = 2 * atan(FZ / (P(11) * P(1) * L(3)));
x2 = P(10) * P(1) * sin(x1) .* (1-P(12) * abs(IA)) * L(4) * L(5);
By = x2 ./ (Cy*Dy);

x3 = By .* alphaY;
FY = Dy .* sin(Cy * atan(x3-Ey .* (x3-atan(x3)))) + Svy;
% End of Brennan's section

alpha_star = tan(alpha) .* sign(Vcx); %4.E3, may be replaced with alpha_star = -(Vcy ./ abs(Vcx))

SHt = Q(1) + (Q(2) .* dfz) + (Q(3) + (Q(4) .* dfz)) .* sin(IA); % 4.E35

SHf = Shy + (Svy ./ (Kya + 0.1)); % 4.E38-39

alpha_r = alpha_star .* SHf; % 4.E37

Br = Q(20) .* By .* Cy; % 4.E45

Cr = 1; % 4.E46 Using the zeta definition on page 185

Dr = FZ .* 8 .* ((Q(21) + (Q(22) .* dfz)) + (Q(23) + (Q(24) .* dfz)) .* sin(IA)) .* (Vcx ./ (Vc + 0.1)); % 4.E47 + E4, E6-E7

MZr0 = Dr .* cos(Cr .* atan(Br .* alpha_r)); % 4.E36

alpha_t = alpha_star + SHt; % 4.E34

Bt = (Q(5) + (Q(6) .* dfz) + (Q(7) .* dfz.^2)) .* (1 + (Q(8) .* sin(IA)) + (Q(9) .* abs(sin(IA)))); % 4.E40

Ct = Q(10); % 4.E41

Dt0 = FZ .* (8 ./ FZ) .* (Q(11) + (Q(12) .* dfz)) .* sign(Vcx); % 4.E42

Dt = Dt0 .* (1 + (Q(13) .* sin(IA)) + (Q(14) .* (sin(IA).^2))); % 4.E43

Et = (Q(15) + (Q(16) .* dfz) + (Q(17) .* dfz.^2)) .* (1 + (Q(18) + (Q(19) .* sin(IA))) .* (2/pi) .* atan(Bt .* Ct .* alpha_t)); % 4.E44

t0 = Dt .* cos(Ct .* atan((Bt .* alpha_t) - Et .* ((Bt .* alpha_t) - atan(Bt .* alpha_t)))) .* (Vcx ./ (Vc + 0.1)); % 4.E33, Definition of cos'(alpha) found in 4.E6-4.E7

Kzao = Dt0 .* Kyao; % 4.E48 I'm going to be honest, I don't know what this does but it's the book so *shrug*

Kzyo = FZ .* 8 .* (Q(23) + (Q(24) .* dfz)) - Dt0 .* Kyy0; %4.E49 Again, stuff just shows up without explanation.  May incorporate values from the previous pacejka.m?

MZ0_prime = -t0 .* FY; % 4.E32

MZ0 = MZ0_prime + MZr0; % 4.E31

end
