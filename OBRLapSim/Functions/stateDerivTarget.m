function [target] = stateDerivTarget(Car,speed,curvature,accelX)
% *************************************************************************
% FUNCTION NAME:
%   stateDerivTarget
%
% DESCRIPTION:
%   Target for solver. 
%   Solver will try to get state derivatives to converge to these values
%
% INPUTS:
%   Car - car data struct
%   speed - (m/s)
%   curvature - (1/m)
%   accelX - longitudinal acceleration (m/s^2)
%
% OUTPUTS:
%   target - array of target values for each state derivative
% 
% KNOW ISSUES:
%
% CHANGE LOG:
%   2018-04-02: Initial revision
% *************************************************************************


target = [speed^2*curvature; 0; 0; 0; 0; 0; 0; 0; 0; accelX/Car.Chassis.tyreRadius; accelX/Car.Chassis.tyreRadius; accelX/Car.Chassis.tyreRadius; accelX/Car.Chassis.tyreRadius; 0; 0; 0; 0];
end

