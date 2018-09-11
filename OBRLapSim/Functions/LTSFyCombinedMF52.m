function [Fy] = LTSFyCombinedMF52(Car, slipAngle,SR, Fz, inclinationAngle)
% *************************************************************************
% FUNCTION NAME:
%   LTSFyCombinedMF52
%
% DESCRIPTION:
%   Computes tyre Fy according to MF5.2 tyre model
%
% INPUTS:
%   Car - car data struct
%   slipAngle - wheel slip angle (deg) 
%   SR - slip ratio (-) 
%   Fz - normal load (N) 
%   inclinationAngle - wheel inclination angle (deg)
%
% OUTPUTS:
%   Fy - tyre lateral force (N)
% 
% KNOW ISSUES:
%
% CHANGE LOG:
%   2018-04-02: Initial revision
% *************************************************************************

%Global Tyre data%
FZ0 = Car.Tyre.FZ0;
% Scaling factors%
LFZO    = Car.Tyre.LFZO;
LCY     = Car.Tyre.LCY;
LMUY    = Car.Tyre.LMUY;
LEY     = Car.Tyre.LEY;
LKY     = Car.Tyre.LKY;
LHY     = Car.Tyre.LHY;
LVY     = Car.Tyre.LVY;
LGAY    = Car.Tyre.LGAY;
LYKA    = Car.Tyre.LYKA;
LVYKA   = Car.Tyre.LVYKA;


%Input variable conversion%

K=SR;
ALPHA  =  slipAngle*pi/180;
FZ     =  Fz;
GAMMA  =  inclinationAngle*pi/180;

%Fy calculations%

GAMMAY = GAMMA .* LGAY; %31 (%48 lgay=lg
FZ0PR  = FZ0  .*  LFZO; %15,  NEED LFZO NOT LFZ0 TO MATCH TIRE PROP FILE
DFZ    = (FZ-FZ0PR) ./ FZ0PR; %14,  (%30)

%Fy pure coefficients%

PCY1    = Car.Tyre.PCY1;
PDY1    = Car.Tyre.PDY1;
PDY2    = Car.Tyre.PDY2;
PDY3    = Car.Tyre.PDY3;
PEY1    = Car.Tyre.PEY1;
PEY2    = Car.Tyre.PEY2;
PEY3    = Car.Tyre.PEY3;
PEY4    = Car.Tyre.PEY4;
PKY1    = Car.Tyre.PKY1;
PKY2    = Car.Tyre.PKY2;
PKY3    = Car.Tyre.PKY3;
PHY1    = Car.Tyre.PHY1;
PHY2    = Car.Tyre.PHY2;
PHY3    = Car.Tyre.PHY3;
PVY1    = Car.Tyre.PVY1;
PVY2    = Car.Tyre.PVY2;
PVY3    = Car.Tyre.PVY3;
PVY4    = Car.Tyre.PVY4;

%Fy ombined Coefficients%

RBY1    = Car.Tyre.RBY1;
RBY2    = Car.Tyre.RBY2;
RBY3    = Car.Tyre.RBY3;
RCY1    = Car.Tyre.RCY1;
REY1    = Car.Tyre.REY1;
REY2    = Car.Tyre.REY2;
RHY1    = Car.Tyre.RHY1;
RHY2    = Car.Tyre.RHY2;
RVY1    = Car.Tyre.RVY1;
RVY2    = Car.Tyre.RVY2;
RVY3    = Car.Tyre.RVY3;
RVY4    = Car.Tyre.RVY4;
RVY5    = Car.Tyre.RVY5;
RVY6    = Car.Tyre.RVY6; 
%c
%c -- lateral force (pure side slip)
%c
SHY     = (PHY1+PHY2 .* DFZ) .* LHY + PHY3 .* GAMMAY; %38,  (%55)
ALPHAY  = ALPHA+SHY;  %30 (%47)
CY      = PCY1 .* LCY;  %32 (%49)
MUY     = (PDY1+PDY2 .* DFZ) .* (1.0-PDY3 .* GAMMAY.^2) .* LMUY; %34 (%51)
DY      = MUY .* FZ; %33 (%50)
KY      = PKY1 .* FZ0 .* sin(2.0 .* atan(FZ ./ (PKY2 .* FZ0 .* LFZO))) .* (1.0-PKY3 .* abs(GAMMAY)) .* LFZO .* LKY; %36 (%53)
BY      = KY ./ (CY .* DY);  %37 (%54)
% NOTE, PER SVEN @TNO: "SIGN(ALPHAY)"IS CORRECT AS IN DOCUMENTATION & BELOW; IT'S NOT SUPPOSED TO BE "SIGN(GAMMAY)"
EY      = (PEY1+PEY2 .* DFZ) .* (1.0-(PEY3+PEY4 .* GAMMAY) .* sign(ALPHAY)) .* LEY; %35 (%52)
% NOTE: LVY MULTIPLIES ONLY PVY1&2 IN DOCUMENTATION; ORIG VERSION MULT ALL TERMS
SVY     = FZ .* ((PVY1+PVY2 .* DFZ) .* LVY+(PVY3+PVY4 .* DFZ) .* GAMMAY) .* LMUY; %39 (%56)
FY0     = DY .* sin(CY .* atan(BY .* ALPHAY-EY .* (BY .* ALPHAY-atan(BY .* ALPHAY))))+SVY; %29 (%46)

%Fy Combined following PAC2002 ADAMS help file%

SHYK= RHY1+RHY2*DFZ; %74%
KS= K + SHYK;%69%
BYK=RBY1*(cos(atan(RBY2*(ALPHA-RBY3))))*LYKA;%70%
CYK=RCY1;%71%
EYK=REY1+REY2*DFZ;%73%
%documentation states EYK<=1, therefore%
if EYK>1
    EYK=1;
end
DYK= FY0/(cos(CYK*atan(BYK*SHYK-EYK*(BYK*SHYK-atan(BYK*SHYK)))));%72%
DVYK=MUY*FZ*(RVY1+RVY2*DFZ+RVY3*GAMMA)*cos(atan(RVY4*ALPHA));%76%
SVYK=DVYK*sin(RVY5*atan(RVY6*K))*LVYKA;%75%
FY=DYK*cos(CYK*atan(BYK*KS-EYK*(BYK*KS-atan(BYK*KS))))+SVYK;%68%

Fy=FY;
