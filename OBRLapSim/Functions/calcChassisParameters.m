function [Chassis] = calcChassisParameters(Chassis)
% *************************************************************************
% FUNCTION NAME:
%   calcChassisParameters
%
% DESCRIPTION:
%   Calculates additional chassis parameters based on provided values.
%   This function should be used if a value in the Car structure is
%   changed.
%
% INPUTS:
%   Chassis - Struct that contains chassis information
%
% OUTPUTS:
%   Chassis - input struct with addiotnal information added
% 
% KNOW ISSUES:
%
% CHANGE LOG:
%   2018-04-02: Initial revision
% *************************************************************************




Chassis.totalMass = Chassis.suspMass+2*Chassis.fUnsuspMass+2*Chassis.rUnsuspMass;

% Wheelrates
Chassis.fWheelrate = Chassis.fSpring*Chassis.fMotionRatio^2;
Chassis.rWheelrate = Chassis.rSpring*Chassis.rMotionRatio^2;

% CG location
Chassis.a = Chassis.wheelbase*(1-Chassis.frontWD); %front axle to CG
Chassis.b = Chassis.wheelbase*(Chassis.frontWD); %rear axle to CG

% Static wheel loads
Chassis.staticFzFL = 9.81*Chassis.totalMass*Chassis.frontWD*Chassis.leftWD;
Chassis.staticFzFR = 9.81*Chassis.totalMass*Chassis.frontWD*(1-Chassis.leftWD);
Chassis.staticFzRL = 9.81*Chassis.totalMass*(1-Chassis.frontWD)*Chassis.leftWD;
Chassis.staticFzRR = 9.81*Chassis.totalMass*(1-Chassis.frontWD)*(1-Chassis.leftWD);

% Lateral distances from CG to contact patches
Chassis.cgCenterlineDist = (Chassis.leftWD-0.5)/(Chassis.frontWD/Chassis.fTrack+(1-Chassis.frontWD)/Chassis.rTrack);
Chassis.centerlineFL = Chassis.fTrack/2-Chassis.cgCenterlineDist; %lateral distance LF tire to CG
Chassis.centerlineFR = Chassis.fTrack/2+Chassis.cgCenterlineDist; %lateral distance RF tire to CG
Chassis.centerlineRL = Chassis.rTrack/2-Chassis.cgCenterlineDist; %lateral distance LR tire to CG
Chassis.centerlineRR = Chassis.rTrack/2+Chassis.cgCenterlineDist; %lateral distance RR tire to CG

% CG height and roll moment arm
Chassis.H = (Chassis.suspMass*Chassis.SMCG + Chassis.fUnsuspMass*Chassis.fUMCG + Chassis.rUnsuspMass*Chassis.rUMCG)/(Chassis.suspMass+Chassis.fUnsuspMass+Chassis.rUnsuspMass); %CoG height [m]
Chassis.h = Chassis.SMCG - (((Chassis.rRC-Chassis.fRC)/Chassis.wheelbase)*Chassis.a + Chassis.fRC); %Distance from CoG to roll axis [m]

% Anti Pitch
Chassis.antisquat = tan(Chassis.antisquatAngle*pi/180)/(Chassis.H/Chassis.wheelbase);
Chassis.antidive = tan(Chassis.fAntidiveAngle*pi/180)/(Chassis.H/Chassis.wheelbase)*Chassis.brakeBias+tan(Chassis.rAntidiveAngle*pi/180)/(Chassis.H/Chassis.wheelbase)*(1-Chassis.brakeBias);

end