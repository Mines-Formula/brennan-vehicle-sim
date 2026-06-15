function FY = pacejka(P, L, FZ, IA, alpha)

%inputs 19 P coefficients,
%Estimated coefficients are as follows:
%P=[250,1.4,2.4,-0.25,3,-0.1,-1.5,0,0,-30.5,1.15,1,0,0,-0.128,0,0,0,1.43];
%L=[0.62,1,1,1,1,1,1,1];

dfz = (FZ - P(1))/P(1);
Svy = FZ.*(P(16)+P(17)*dfz+(P(18)+P(19)*dfz).*IA)*L(8)*L(5);
Ey = (P(6)+P(7)*dfz).*(1-(P(8)+P(9)*IA).*sign(alpha))*L(7);
Shy = (P(13)+P(14)*dfz+P(15)*IA)*L(6);
alphaY = alpha + Shy;
Cy = P(2)*L(2);
Dy = FZ.*(P(3)+P(4)*dfz).*(1-P(5)*IA.^2)*L(1);
x1 = 2*atan(FZ/(P(11)*P(1)*L(3)));
x2 = P(10)*P(1)*sin(x1).*(1-P(12)*abs(IA))*L(4)*L(5);
By = x2 ./ (Cy*Dy);

x3 = By.*alphaY;
FY = Dy.*sin(Cy*atan(x3-Ey.*(x3-atan(x3))))+Svy;

