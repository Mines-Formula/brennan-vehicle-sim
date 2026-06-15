function SA = findSlip(FZ1, FZtot, FY, IA1, IA2,toeEff,P,L)
%FZ1 is FZ on outside tire
%FZtot is FZ on axle
%FY is required lateral grip at axle
%IA1 is IA on outside tire in degrees
%IA2 is IA on inside tire in degrees
%toeEff is toe angle. For rear axle, = to static toe. For front, = to
%static toe at 100% ackerman. Represents difference in inside vs outside SA.
%Positive = toe in (degrees)
%P is 19 pacejka coeff
%L is lamba factors
IA1 = IA1*pi/180;
IA2 = IA2*pi/180*-1; %switch signs for inside tire
toeEff = toeEff*pi/180;

%outputs slip angle that satifies requried lateral grip
%if required grip is above possible grip, output will be upper bound b

FY = -FY;
FZ2 = FZtot-FZ1;  %inside tire load
a = 1*pi/180;     %lower bound
b = 10*pi/180;     %upper bound
SAguess1 = (a+b)/2 + toeEff; %initial guess for outside tire
SAguess2 = (a+b)/2 - toeEff;
n = 0;

while abs(pacejka(P,L,FZ1,IA1,SAguess1)+pacejka(P,L,FZ2,IA2,SAguess2)-FY) > 0.01 && n < 1000
    if pacejka(P,L,FZ1,IA1,SAguess1)+pacejka(P,L,FZ2,IA2,SAguess2) > FY
        a = (SAguess1+SAguess2)/2; %increase SA guess
    else
        b = (SAguess1+SAguess2)/2; %decrease SA guess
    end
    SAguess1 = (a+b)/2 + toeEff; %update guess for outside tire
    SAguess2 = (a+b)/2 - toeEff;
    n = n +1; %ensures loop doesn't run forever
end

SA = (SAguess1+SAguess2)/2*180/pi; %output in degrees
        



