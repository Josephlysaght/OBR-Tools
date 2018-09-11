function [FLTyre, FRTyre, RLTyre, RRTyre] = calcTyreSlip(Car, speed, curvature, yawAngle, FLTyre, FRTyre, RLTyre, RRTyre)
% *************************************************************************
% FUNCTION NAME:
%   calcTyreSlip
%
% DESCRIPTION:
%   This function calculates the slip angle of each wheel
%
% INPUTS:
%   Car - car data struct
%   speed - (m/s)
%   curvature - (1/m)
%   yawAngle - (deg)
%   FLTyre - tyre data struct
%   FRTyre - ''
%   RLTyre - ''
%   RRTyre - ''
%
% OUTPUTS:
%   FLTyre - tyre data struct
%   FRTyre - ''
%   RLTyre - ''
%   RRTyre - '' 
% 
% KNOW ISSUES:
%
% CHANGE LOG:
%   2018-04-02: Initial revision
% *************************************************************************



%Setting Velocity vectors
Velocity = speed*[cos(yawAngle*pi/180) -sin(yawAngle*pi/180) 0]; %This comes from [u v 0]
yawRate = [0 0 speed*curvature]; %This comes from [0 0 r]

%Location of wheels relative to CG 
positionFL = [Car.Chassis.a Car.Chassis.centerlineFL 0];
positionFR = [Car.Chassis.a -Car.Chassis.centerlineFR 0];
positionRL = [-Car.Chassis.b Car.Chassis.centerlineRL 0];
positionRR = [-Car.Chassis.b -Car.Chassis.centerlineRR 0];

%Using cross product to calculate tangential speed
VFL = Velocity + cross(yawRate,positionFL);
VFR = Velocity + cross(yawRate,positionFR);
VRL = Velocity + cross(yawRate,positionRL);
VRR = Velocity + cross(yawRate,positionRR);

%Calculating slip angle on each wheel
FLTyre.slipAngle = 180/pi*(atan(-VFL(2)/VFL(1))) + FLTyre.sigma;
FRTyre.slipAngle = 180/pi*(atan(-VFR(2)/VFR(1))) + FRTyre.sigma;
RLTyre.slipAngle = 180/pi*(atan(-VRL(2)/VRL(1))) + RLTyre.sigma;
RRTyre.slipAngle = 180/pi*(atan(-VRR(2)/VRR(1))) + RRTyre.sigma;

%Calculate slip ratio
FLTyre.slipRatio = FLTyre.angVel*Car.Chassis.tyreRadius/VFL(1)-1;
FRTyre.slipRatio = FRTyre.angVel*Car.Chassis.tyreRadius/VFR(1)-1;
RLTyre.slipRatio = RLTyre.angVel*Car.Chassis.tyreRadius/VRL(1)-1;
RRTyre.slipRatio = RRTyre.angVel*Car.Chassis.tyreRadius/VRR(1)-1;

end