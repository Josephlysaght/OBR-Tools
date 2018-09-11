function [FLcamber, FRcamber, RLcamber, RRcamber, sigmaFL, sigmaFR, sigmaRL, sigmaRR] = kinematics(Car, FLWheelDisp, FRWheelDisp, RLWheelDisp, RRWheelDisp, roll, steering)
% *************************************************************************
% FUNCTION NAME:
%   kinematics
%
% DESCRIPTION:
%   kinematics Polynomials fitted in createCarStruct.m are evalauted here at
%   every iteration step to obtain camber and toe for each wheel
%
%   For rear wheels, built-in polyfit function was used, while front
%   kinematics are evaluated using polyfitn from MATLAB Central to obtain
%   camber and toe as a function of heave and steer
%
%   Front setup allows for asymmetric camber/toe
%   Sign of RHS toe needs to be changed according to convention (LHS: toe out
%   is positive, RHS toe in is positive)
%
% INPUTS:
%   Car - car data struct
%   FLWheelDisp, FRWheelDisp, RLWheelDisp, RRWheelDisp - wheel displacements (m) 
%   roll - (deg) 
%   steering - steering wheel angle (deg)
%
% OUTPUTS:
%   FLcamber, FRcamber, RLcamber, RRcamber - wheel cambers 
%   sigmaFL, sigmaFR, sigmaRL, sigmaRR - wheel angles relative to chassis
% 
% KNOW ISSUES:
%
% CHANGE LOG:
%   2018-04-02: Initial revision
% *************************************************************************


%% Front Kinematics

% Calculate final camber and toe values by evaluating polynomial against
% individual wheel translations and steering angle
% Kinematic maps are done in length units of mm so wheel displacemnets need
% to be converted from m to mm

FLcamber = polyvaln( Car.Kin.camber_poly_fl , [FLWheelDisp*1000,steering] ) - roll + Car.Chassis.staticCamberFL;
FRcamber = polyvaln( Car.Kin.camber_poly_fr , [FRWheelDisp*1000,steering] ) + roll + Car.Chassis.staticCamberFR;

sigmaFL = polyvaln( Car.Kin.toe_poly_fl , [FLWheelDisp*1000,steering] ) + Car.Chassis.toeFL ;
sigmaFR = -polyvaln( Car.Kin.toe_poly_fr , [FRWheelDisp*1000,steering] ) - Car.Chassis.toeFR ;

%% Rear Kinematics

% Calculate final camber and toe values by evaluating polynomial against
% individual wheel translations

% Subtract or add roll to obtain absolute camber, relative to road
RLcamber = polyval(Car.Kin.camber_poly_r,RLWheelDisp*1000) - roll + Car.Chassis.staticCamberRL;
RRcamber = polyval(Car.Kin.camber_poly_r,RRWheelDisp*1000) + roll + Car.Chassis.staticCamberRR;

sigmaRL = polyval(Car.Kin.toe_poly_r,RLWheelDisp*1000) + Car.Chassis.toeRL;
sigmaRR = -polyval(Car.Kin.toe_poly_r,RRWheelDisp*1000) - Car.Chassis.toeRR;

end