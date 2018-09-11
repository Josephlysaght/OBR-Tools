function [fCl, rCl, Cd] = aeroCoefficients(Car, speed, yaw, roll, fRH, rRH, averageSteeredAngle)
% *************************************************************************
% FUNCTION NAME:
%   aeroCoefficients
%
% DESCRIPTION:
%   Returns aero coefficients based on vehicle attitude
%
% INPUTS:
%   Car - car data struct
%   speed - (m/s)
%   yaw - (deg)
%   roll - (deg)
%   fRH - front ride height (m)
%   rRH - rear ride height (m)
%   averageSteeredAngle - average of right and left front steer angles (deg)
%
% OUTPUTS:
%   outputStruct - return the input struct with the data from the file
%   added
% 
% KNOW ISSUES:
%   steering and yaw neglected to stay in range of aeromap data
%
% CHANGE LOG:
%   2018-04-02: Initial revision
% *************************************************************************

if Car.Aero.map
    % Aero map ride heights are in mm 
    fCl = Car.Aero.aeroMapLiftScaling * AeroInterp(Car.Aero.YRegCzf, Car.Aero.dims, [ fRH*1000, rRH*1000, 0*yaw, 0*averageSteeredAngle, roll]);
    rCl = Car.Aero.aeroMapLiftScaling * AeroInterp(Car.Aero.YRegCzr, Car.Aero.dims, [ fRH*1000, rRH*1000, 0*yaw, 0*averageSteeredAngle, roll]);
    Cd = Car.Aero.aeroMapDragScaling * AeroInterp(Car.Aero.YRegCx, Car.Aero.dims, [ fRH*1000, rRH*1000, 0*yaw, 0*averageSteeredAngle, roll]);
else
    fCl = Car.Aero.fCl;
    rCl = Car.Aero.rCl;
    Cd = Car.Aero.Cd;
end
end