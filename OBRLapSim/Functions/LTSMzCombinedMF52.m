function [Mz] = LTSMzCombinedMF52(Car, slipAngle,SR, Fz, Camber)
% *************************************************************************
% FUNCTION NAME:
%   LTSMzCombinedMF52
%
% DESCRIPTION:
%   Computes tyre Mz according to MF5.2 tyre model
%
% INPUTS:
%   Car - car data struct
%   slipAngle - wheel slip angle (deg) 
%   SR - slip ratio (-) 
%   Fz - normal load (N) 
%   inclinationAngle - wheel inclination angle (deg)
%
% OUTPUTS:
%   Fx - tyre aligning moment (Nm)
% 
% KNOW ISSUES:
%
% CHANGE LOG:
%   2018-04-02: Initial revision
% *************************************************************************

%Global Tyre data%
FZ0 = Car.Tyre.FZ0;
% Scaling factors%REVIEW!!!!!
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
LCX     = Car.Tyre.LCX;
LMUX    = Car.Tyre.LMUX;
LEX     = Car.Tyre.LEX;
LHX     = Car.Tyre.LHX;
LVX     = Car.Tyre.LVX;
LGAX    = Car.Tyre.LGAX;
LKXK    = Car.Tyre.LKXK;
LXA    = Car.Tyre.LXA;
LTR  = Car.Tyre.LTR;
LRES  = Car.Tyre.LRES;
LGAZ = Car.Tyre.LGAZ;
LXAL = Car.Tyre.LXAL;
LS = Car.Tyre.LS;
LSGKP  = Car.Tyre.LSGKP;
LSGAL = Car.Tyre.LSGAL;
LGYR  = Car.Tyre.LGYR;


%Input variable conversion%

K=SR;
ALPHA  =  slipAngle*pi/180;
FZ     =  Fz;
GAMMA  =  Camber*pi/180;

GAMMAY = GAMMA .* LGAY; %31 (%48 lgay=lg
GAMMAZ = GAMMA .* LGAZ; %47 (%63 lgaz = lg
FZ0PR  = FZ0  .*  LFZO; %15,  NEED LFZO NOT LFZ0 TO MATCH TIRE PROP FILE
DFZ    = (FZ-FZ0PR) ./ FZ0PR; %14,  (%30)

%Mz pure coefficients%
QBZ1    = Car.Tyre.QBZ1 ;
QBZ2    = Car.Tyre.QBZ2;
QBZ3    = Car.Tyre.QBZ3;
QBZ4    = Car.Tyre.QBZ4;
QBZ5    = Car.Tyre.QBZ5;
QBZ9    = Car.Tyre.QBZ9;
QBZ10   = Car.Tyre.QBZ10;
QCZ1    = Car.Tyre.QCZ1;
QDZ1    = Car.Tyre.QDZ1;
QDZ2    = Car.Tyre.QDZ2;
QDZ3    = Car.Tyre.QDZ3;
QDZ4    = Car.Tyre.QDZ4;
QDZ6    = Car.Tyre.QDZ6;
QDZ7    = Car.Tyre.QDZ7;
QDZ8    = Car.Tyre.QDZ8;
QDZ9    = Car.Tyre.QDZ9;
QEZ1    = Car.Tyre.QEZ1;
QEZ2    = Car.Tyre.QEZ2;
QEZ3    = Car.Tyre.QEZ3;
QEZ4    = Car.Tyre.QEZ4;
QEZ5    = Car.Tyre.QEZ5;
QHZ1    = Car.Tyre.QHZ1;
QHZ2    = Car.Tyre.QHZ2;
QHZ3    = Car.Tyre.QHZ3;
QHZ4    = Car.Tyre.QHZ4;

%C
%C -- ALIGNING TORQUE (PURE SIDE SLIP)
%C

SHY     = (PHY1+PHY2 .* DFZ) .* LHY + PHY3 .* GAMMAY; %38,  (%55)
SVY     = FZ .* ((PVY1+PVY2 .* DFZ) .* LVY+(PVY3+PVY4 .* DFZ) .* GAMMAY) .* LMUY; %39 (%56)
ALPHAY  = ALPHA+SHY;  %30 (%47)

SHT    = QHZ1+QHZ2 .* DFZ+(QHZ3+QHZ4 .* DFZ) .* GAMMAZ; %52 ( %68)
ALPHAT = ALPHA+SHT;  %43 (%59)
KY      = PKY1 .* FZ0 .* sin(2.0 .* atan(FZ ./ (PKY2 .* FZ0 .* LFZO))) .* (1.0-PKY3 .* abs(GAMMAY)) .* LFZO .* LKY; %36 (%53)
% NOTE: PER SVEN, "EQUATION 45 IS WRONG DOCUMENTATION,
% THERE IT SHOULD BE SHF INSTEAD OF SHR"
SHF    = SHY+SVY ./ KY; %46 (%62)
ALPHAR = ALPHA+SHF; %45 (%61)

BT = (QBZ1+QBZ2 .* DFZ+QBZ3 .* DFZ.^2) .* (1.0+QBZ4 .* GAMMAZ+QBZ5 .* abs(GAMMAZ)) .* LKY ./ LMUY; %48 (%64)
CT = QCZ1; %49 (%65)
DT = FZ .* (QDZ1+QDZ2 .* DFZ) .* (1.0+QDZ3 .* GAMMAZ+QDZ4 .* GAMMAZ.^2) .* (R0 ./ FZ0) .* LTR; %50 (%66)
% NOTE: EQUATION FOR ET HAS CHANGED FROM PAC97 EQUATION; 2/PI TERM IS NEW.
ET = (QEZ1+QEZ2 .* DFZ+QEZ3 .* DFZ.^2) .* (1.0+(QEZ4+QEZ5 .* GAMMAZ) .* (2/pi) .* atan(BT .* CT .* ALPHAT)); %51 (%67)
CY      = PCY1 .* LCY;  %32 (%49)
MUY     = (PDY1+PDY2 .* DFZ) .* (1.0-PDY3 .* GAMMAY.^2) .* LMUY; %34 (%51)

DY      = MUY .* FZ; %33 (%50)
BY      = KY ./ (CY .* DY);  %37 (%54)
BR = QBZ9 .* LKY ./ LMUY+QBZ10 .* BY .* CY;  %53 (%69)
% NOTE: LRES MULTIPLIES EVERYTHING IN ORIG EQN; BELOW MATCHES DOCUMENTATION
DR = FZ .* ((QDZ6+QDZ7 .* DFZ) .* LRES+(QDZ8+QDZ9 .* DFZ) .* GAMMAZ) .* R0 .* LMUY; %54 (%70 LRES=LMR)
TRAIL = DT .* cos(CT .* atan(BT .* ALPHAT-ET .* (BT .* ALPHAT-atan(BT .* ALPHAT)))) .* cos(ALPHA); %42 (%58)
MZR = DR .* cos(atan(BR .* ALPHAR)) .* cos(ALPHA); %44 (%60)
EY      = (PEY1+PEY2 .* DFZ) .* (1.0-(PEY3+PEY4 .* GAMMAY) .* sign(ALPHAY)) .* LEY; %35 (%52)
% NOTE: LVY MULTIPLIES ONLY PVY1&2 IN DOCUMENTATION; ORIG VERSION MULT ALL TERMS
SVY     = FZ .* ((PVY1+PVY2 .* DFZ) .* LVY+(PVY3+PVY4 .* DFZ) .* GAMMAY) .* LMUY; %39 (%56)
FY0     = DY .* sin(CY .* atan(BY .* ALPHAY-EY .* (BY .* ALPHAY-atan(BY .* ALPHAY))))+SVY; %29 (%46)
MZ0= -TRAIL .* FY0 + MZR;  %41 (%57)
%MZ = MZ0; %40
MZ = -MZ0; %40

