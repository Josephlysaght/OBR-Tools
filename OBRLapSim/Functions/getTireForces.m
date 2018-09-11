function [FLTyre,FRTyre,RLTyre,RRTyre] = getTireForces(Car,FLTyre,FRTyre,RLTyre,RRTyre)
% *************************************************************************
% FUNCTION NAME:
%   getTireForces
%
% DESCRIPTION:
%   Takes tyre classes and return with updated force values
%   Uses ISO tyre coordinate system
%
% INPUTS:
%   tyre structs
%
% OUTPUTS:
%   tyre structs
% 
% KNOW ISSUES:
%
% CHANGE LOG:
%   2018-04-02: Initial revision
% *************************************************************************



%Tranform forces and angles from vehicle cordinate system to tyre
%coordinate system

% ISO tyre coordinate system
% Y axis, slip angle, and inclination angle  directions flipped for right
% side to mirror tyre model

FLFz=FLTyre.Fz;
FRFz=FRTyre.Fz;
RLFz=RLTyre.Fz;
RRFz=RRTyre.Fz;

%Convert camber to inclination angles
FLinclAngle = -FLTyre.camber;
FRinclAngle = -FRTyre.camber;
RLinclAngle = -RLTyre.camber;
RRinclAngle = -RRTyre.camber;

%Covert slip angles
FLslipAngle = -FLTyre.slipAngle;
FRslipAngle = FRTyre.slipAngle;
RLslipAngle = -RLTyre.slipAngle;
RRslipAngle = RRTyre.slipAngle;

%Get Forces
FLFy=LTSFyCombinedMF52(Car,FLslipAngle,FLTyre.slipRatio,FLFz,FLinclAngle);
FRFy=LTSFyCombinedMF52(Car,FRslipAngle,FRTyre.slipRatio,FRFz,FRinclAngle);
RLFy=LTSFyCombinedMF52(Car,RLslipAngle,RLTyre.slipRatio,RLFz,RLinclAngle);
RRFy=LTSFyCombinedMF52(Car,RRslipAngle,RRTyre.slipRatio,RRFz,RRinclAngle);

FLFx=LTSFxCombinedMF52(Car,FLslipAngle,FLTyre.slipRatio,FLFz,FLinclAngle);
FRFx=LTSFxCombinedMF52(Car,FRslipAngle,FRTyre.slipRatio,FRFz,FRinclAngle);
RLFx=LTSFxCombinedMF52(Car,RLslipAngle,RLTyre.slipRatio,RLFz,RLinclAngle);
RRFx=LTSFxCombinedMF52(Car,RRslipAngle,RRTyre.slipRatio,RRFz,RRinclAngle);

%Convert forces from tyre to vehicle coordinate system
FLTyre.Fy = FLFy;
FRTyre.Fy = -FRFy;
RLTyre.Fy = RLFy;
RRTyre.Fy = -RRFy;

FLTyre.Fx = FLFx;
FRTyre.Fx = FRFx;
RLTyre.Fx = RLFx;
RRTyre.Fx = RRFx;

end
