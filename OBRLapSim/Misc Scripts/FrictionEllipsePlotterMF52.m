%Create Tyre data structure in car structure%
% Car.Tyre = struct();
% Car.Tyre = addParametersFromFile(Car.Tyre,'TyreCoefficientsHoosierLC0_LatCombinedFrom13R25B.xlsx');

Fz=1000;
Camber=0;

SRinc=0.01;
SRmax=0.2;
SAinc=0.01;
SAmax=15;
SR=-SRmax:SRinc:SRmax;
SA=-SAmax:SAinc:SAmax;
Fx=zeros(length(SR),length(SA));
Fy=zeros(length(SR),length(SA));

for i=1:length(SR)
    for j=1:length(SA)
        Fx(i,j) = LTSFxCombinedMF52(Car, SA(j), SR(i), Fz, Camber);
        Fy(i,j)=LTSFyCombinedMF52(Car,SA(j),SR(i), Fz, Camber);        
    end
end

figure
hold on
for i=1:2:length(SR)
    plot(Fy(i,:),Fx(i,:),'k')
end
for j=1:100:length(SA)
    plot(Fy(:,j),Fx(:,j),'k')
end
xlabel('Fy (N)')
ylabel('Fx (N)')
title(strcat('Hoosier 6.0/18.0-10 LC0 12psi Fz =', 32, num2str(Fz), 32, 'N Friction Ellipse'))