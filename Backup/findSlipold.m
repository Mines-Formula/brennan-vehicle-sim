function SA = findSlipold(FZ1, FZtot, FY, IA1, IA2,P,L)
%FZ1 is FZ on outside tire
%FZtot is FZ on axle
%FY is required lateral grip at axle
%IA1 is IA on outside tire in degrees
%IA2 is IA on inside tire in degrees
%Toe is toe angle
%P is 19 pacejka coeff
%L is lamba factors
IA1 = IA1*pi/180;
IA2 = IA2*pi/180*-1; %switch signs for inside tire

%outputs slip angle that satifies requried lateral grip
%if required grip is above possible grip, output will be upper bound b

FY = -FY;
FZ2 = FZtot-FZ1;  %inside tire load
a = 1*pi/180;     %lower bound
b = 7.5*pi/180;     %upper bound
SAguess = (a+b)/2; %initial guess for outside tire
n = 0;

while abs(pacejka(P,L,FZ1,IA1,SAguess)+pacejka(P,L,FZ2,IA2,SAguess)-FY) > 0.01 && n < 1000
    if pacejka(P,L,FZ1,IA1,SAguess)+pacejka(P,L,FZ2,IA2,SAguess) > FY
        a = SAguess; %increase SA guess
    else
        b = SAguess; %decrease SA guess
    end
    SAguess = (a+b)/2;
    n = n +1; %ensures loop doesn't run forever
end

SA = SAguess*180/pi; %ouput in degrees
        