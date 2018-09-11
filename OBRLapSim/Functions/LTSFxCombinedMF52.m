function [Fx] = LTSFxCombinedMF52(Car, slipAngle, SR, Fz, inclinationAngle)
% *************************************************************************
% FUNCTION NAME:
%   LTSFxCombinedMF52
%
% DESCRIPTION:
%   Computes tyre Fx according to MF5.2 tyre model
%
% INPUTS:
%   Car - car data struct
%   slipAngle - wheel slip angle (deg) 
%   SR - slip ratio (-) 
%   Fz - normal load (N) 
%   inclinationAngle - wheel inclination angle (deg)
%
% OUTPUTS:
%   Fx - tyre longitudinal force (N)
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
LCX     = Car.Tyre.LCX;
LMUX    = Car.Tyre.LMUX;
LEX     = Car.Tyre.LEX;
LHX     = Car.Tyre.LHX;
LVX     = Car.Tyre.LVX;
LGAX    = Car.Tyre.LGAX;
LKXK    = Car.Tyre.LKXK;
LXA    = Car.Tyre.LXA;

%Input variable conversion%
K      =  SR;
FZ     =  Fz;
GAMMA  = inclinationAngle*pi/180;
ALPHA  =  slipAngle*pi/180;

ALPHAX = ALPHA .* LGAX;% not sure
GAMMAX = GAMMA .* LGAX; %31 (%48 lgay=lg
FZ0PR  = FZ0  .*  LFZO; %15,  NEED LFZO NOT LFZ0 TO MATCH TIRE PROP FILE
DFZ    = (FZ-FZ0PR) ./ FZ0PR; %14,  (%30)


PCX1 = Car.Tyre.PCX1;
PDX1 =  Car.Tyre.PDX1;
PDX2 =  Car.Tyre.PDX2;
PDX3 =  Car.Tyre.PDX3;
PEX1 =  Car.Tyre.PEX1;
PEX2 =  Car.Tyre.PEX2;
PEX3 =  Car.Tyre.PEX3;
PEX4 =  Car.Tyre.PEX4;
PKX1 =  Car.Tyre.PKX1;
PKX2 =  Car.Tyre.PKX2;
PKX3 =  Car.Tyre.PKX3;
PHX1 =  Car.Tyre.PHX1;
PHX2 =  Car.Tyre.PHX2;
PVX1 =  Car.Tyre.PVX1;
PVX2 =  Car.Tyre.PVX2;
%COMBINED
RBX1 =  Car.Tyre.RBX1;
RBX2 =  Car.Tyre.RBX2;
RCX1 =  Car.Tyre.RCX1;
REX1 =  Car.Tyre.REX1;
REX2 =  Car.Tyre.REX2;
RHX1 =  Car.Tyre.RHX1;

%--PURE LONGITUDINAL FORCE

SHX=(PHX1+PHX2.*DFZ).*LHX;
KX=K+SHX;

MUX=(PDX1+PDX2.*DFZ).*(1-PDX3.*GAMMAX.^2).*LMUX;
KKX=FZ.*(PKX1+PKX2.*DFZ).*exp(PKX3.*DFZ).*LKXK;%KK=K in text book

EX=(PEX1+PEX2.*DFZ+PEX3.*DFZ.^2).*(1-PEX4.* sign(KX)).*LEX;
CX=PCX1.*LCX;
DX=MUX.*FZ;
BX=(KKX./(CX.*DX));
SVX=FZ.*(PVX1+PVX2.*DFZ).*LVX.*LMUX;

FX0=DX.*sin(CX.*atan(BX.*KX-EX.*(BX.*KX-atan(BX.*KX))))+SVX;
%FX=FX0;


%--COMBINED SLIP

BXA=(RBX1.*cos(atan(RBX2.*K)).*LXA);
CXA=RCX1;
EXA=REX1+REX2.*DFZ;
SHXA=RHX1;

AS=ALPHAX+SHXA;

%DXA=FX0./(cos(CXA./atan(BXA.*SHXA-EXA.*(BXA.*SHXA-atan(BXA*SHXA)))));

%FX=DXA.*cos(CXA.*atan(BXA.*AS-EXA.*(BXA.*AS-atan(BXA.*AS))));

num=cos(CXA.*atan(BXA.*AS-EXA.*(BXA.*AS-atan(BXA.*AS))));
den=cos(CXA.*atan(BXA.*SHXA-EXA.*(BXA.*SHXA-atan(BXA.*SHXA))));

GXA=num./den;

Fx=FX0.*GXA;
