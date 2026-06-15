function MZ = pacejkaMZ(P, L, FZ, IA, alpha)

%inputs 21 P coefficients
%P20 is a modification to P96 that allows shape factor to vary with load
%P21 is a modificatoion that adds a vertical shift with IA
%Estimated coefficients are as follows:
%PM=[250,2.5,0.16,0.12,0,-0.065,-0.8,0,0,4.8,1.8,0,0,0,0.2,-0.01,0,0.4,0,-0.045,250];
%LM=[0.7,1,1,1,1,1,1,1];

dfz = (FZ - P(1))/P(1);
Svy = FZ.*(P(16)+P(17)*dfz+(P(18)+P(19)*dfz).*IA)*L(8)*L(5);
Ey = (P(6)+P(7)*dfz).*(1-(P(8)+P(9)*IA).*sign(alpha))*L(7);
Shy = (P(13)+P(14)*dfz+P(15)*IA)*L(6);
alphaY = alpha + Shy;
Cy = (P(2)+P(20)*sqrt(abs((250-FZ))).*sign(250-FZ))*L(2);
Dy = FZ.*(P(3)+P(4)*dfz).*(1-P(5)*IA.^2)*L(1);
x1 = 2*atan(FZ/(P(11)*P(1)*L(3)));
x2 = P(10)*P(1)*sin(x1).*(1-P(12)*abs(IA))*L(4)*L(5);
By = x2 ./ (Cy.*Dy);

x3 = By.*alphaY;
MZ = Dy.*sin(Cy.*atan(x3-Ey.*(x3-atan(x3))))+Svy+P(21)*IA;