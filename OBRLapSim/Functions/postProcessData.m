function [Output] = postProcessData(Car,Track,Output)
% *************************************************************************
% FUNCTION NAME:
%   postProcessData
%
% DESCRIPTION:
%   %Record outputs and post-process data
%
% INPUTS:
%   Car - car data struct
%   Track - track data struct
%   Output - struct storing output data
%
% OUTPUTS:
%   Output - Struct with added data
% 
% KNOW ISSUES:
%
% CHANGE LOG:
%   2018-04-02: Initial revision
%   2018-05-20: Fixed typo in RRTyreFx variable
% *************************************************************************


%Split throttleOrBrake into seperate outputs
Output.throttle = zeros(length(Track.curvature),1);
Output.brake = zeros(length(Track.curvature),1);
for i=1:length(Output.throttleOrBrake)
    if Output.throttleOrBrake(i)>0
        Output.throttle(i) = Output.throttleOrBrake(i);
    else
        Output.brake(i) = -Output.throttleOrBrake(i);
    end
end
% Pull data from stateVar and place into individual arrays
Output.steering = Output.stateVar(:,1);
Output.yaw = Output.stateVar(:,2);
Output.roll = Output.stateVar(:,3);
Output.heave = Output.stateVar(:,4);
Output.pitch = Output.stateVar(:,5);
Output.FLtyreDeflection = Output.stateVar(:,6);
Output.FRtyreDeflection = Output.stateVar(:,7);
Output.RLtyreDeflection = Output.stateVar(:,8);
Output.RRtyreDeflection = Output.stateVar(:,9);
Output.FLwheelSpeed = Output.stateVar(:,10);
Output.FRwheelSpeed = Output.stateVar(:,11);
Output.RLwheelSpeed = Output.stateVar(:,12);
Output.RRwheelSpeed = Output.stateVar(:,13);


outputData = zeros(length(Track.curvature),40);
Output.accelY = zeros(length(Track.curvature),1);
Output.accelX = zeros(length(Track.curvature),1);
Output.stability = zeros(length(Track.curvature),1);
Output.control = zeros(length(Track.curvature),1);
Output.understeerGradient = zeros(length(Track.curvature),1);
dYaw = zeros(length(Output.stateVar(1,:)),1);
dYaw(2) = 0.01;
dSteering = zeros(length(Output.stateVar(1,:)),1);
dSteering(1) = 0.01;
for i=1:length(Track.curvature)
    [f,Output.accelX(i),outputData(i,:)] = vehicleModel(Car,Output.stateVar(i,:)',Output.speed(i),Track.curvature(i),Output.throttleOrBrake(i));
    Output.accelY(i) = f(1);
    [results] = vehicleModel(Car,(Output.stateVar(i,:)'+dYaw),Output.speed(i),Track.curvature(i),Output.throttleOrBrake(i));
    Output.stability(i) = -sign(Track.curvature(i)+0.0001)*(results(2)-f(2))*Car.Chassis.yawInertia/dYaw(2);
    [results] = vehicleModel(Car,(Output.stateVar(i,:)'+dSteering),Output.speed(i),Track.curvature(i),Output.throttleOrBrake(i));
    Output.control(i) = (results(2)-f(2))*Car.Chassis.yawInertia/dSteering(1);
end

%Organize data from outputData
Output.averageSteeredAngle = outputData(:,1);
Output.FLWheelDisp = outputData(:,2);
Output.FRWheelDisp = outputData(:,3);
Output.RLWheelDisp = outputData(:,4);
Output.RRWheelDisp = outputData(:,5);
Output.FLTyreFz = outputData(:,6);
Output.FRTyreFz = outputData(:,7);
Output.RLTyreFz = outputData(:,8);
Output.RRTyreFz = outputData(:,9);
Output.FLTyreFy = outputData(:,10);
Output.FRTyreFy = outputData(:,11);
Output.RLTyreFy = outputData(:,12);
Output.RRTyreFy = outputData(:,13);
Output.FLTyreFx = outputData(:,14);
Output.FRTyreFx = outputData(:,15);
Output.RLTyreFx = outputData(:,16);
Output.RRTyreFx = outputData(:,17);
Output.FLslipAngle = outputData(:,18);
Output.FRslipAngle = outputData(:,19);
Output.RLslipAngle = outputData(:,20);
Output.RRslipAngle = outputData(:,21);
Output.FLslipRatio = outputData(:,22);
Output.FRslipRatio = outputData(:,23);
Output.RLslipRatio = outputData(:,24);
Output.RRslipRatio = outputData(:,25);
Output.FLcamber = outputData(:,26);
Output.FRcamber = outputData(:,27);
Output.RLcamber = outputData(:,28);
Output.RRcamber = outputData(:,29);
Output.fRH = outputData(:,30);
Output.rRH = outputData(:,31);
Output.gearPos = outputData(:,32);
Output.engineRPM = outputData(:,33);
Output.fCl = outputData(:,34);
Output.rCl = outputData(:,35);
Output.Cd = outputData(:,36);
Output.fAeroLoad = outputData(:,37);
Output.rAeroLoad = outputData(:,38);
Output.drag = outputData(:,39);
Output.fAeroBalance = outputData(:,40);

%Calculate understeer gradient
for n = 1:length(Track.curvature)
    if abs(Output.accelY(n))>0.01
        Output.understeerGradient(n) = (Output.averageSteeredAngle(n)-Car.Chassis.wheelbase*Track.curvature(n)*180/pi)./(Output.accelY(n)/9.81);
    else
        Output.understeerGradient(n) = 0;
    end
end

%Caluculate additional channels
Output.latAccelG = Output.accelY/9.81;
Output.longAccelG = Output.accelX/9.81;
Output.sideSlipAngle = -Output.yaw;
end