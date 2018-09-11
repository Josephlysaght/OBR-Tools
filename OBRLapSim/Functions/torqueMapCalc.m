function [torqueVsSpeed,speed,torque] = torqueMapCalc(Powertrain,tyreRadius,efficiency)
% *************************************************************************
% FUNCTION NAME:
%   torqueMapCalc
%
% DESCRIPTION:
%   This function finds the Torque vs Speed map based on given parameters
%
% INPUTS:
%   Powertrain - struct containing powertrain parameters
%   tyreRadius - tyre radius (m)
%   efficiency - powertrain mechanical efficiency (-)
%
% OUTPUTS:
%   torqueVsSpeed - array with 1st column containg vehicle speed, 2nd column torque and the wheels, and 3rd the gear
%   speed - array of vehicle speed for each engine speed and gear (m/s)
%   torque - array of torque for each engine speed and gear (m/s)
% 
% KNOW ISSUES:
%
% CHANGE LOG:
%   2018-04-02: Initial revision
% *************************************************************************


primaryRatio = Powertrain.primaryRatio;
gearRatio = Powertrain.gearRatio;
finalDriveRatio = Powertrain.finalDriveRatio;
engTorque = Powertrain.torqueCurve;


%Creating zero vectors
speed = zeros(length(engTorque(:,1)),length(gearRatio));
torque = speed;

engTorque(:,2) = efficiency*engTorque(:,2);

%Finding speed and force for each RPM at each gear
for m = 1:length(engTorque(:,1))
    for n = 1:length(gearRatio)
        speed(m,n) = 2*pi*(engTorque(m,1)/(primaryRatio*gearRatio(n)*finalDriveRatio))*tyreRadius/60;
        torque(m,n) = engTorque(m,2)*primaryRatio*gearRatio(n)*finalDriveRatio;
    end
end

%Creating speed array to calculate engine force at each specific car speed
speedInc = 0.1;
speedArray = 0:speedInc:floor(speed(end,end)/speedInc)*speedInc;
interpForce = zeros(length(speedArray),length(gearRatio));
%Finding wheel force for each speed at each gear
for i = 1:length(gearRatio)
    for j = 1:length(speedArray)
        % if speed is lower than available data point use minimum torque
        if speedArray(j)>min(speed(:,i))
            interpForce(j,i) = interp1(speed(:,i),torque(:,i),speedArray(j),'linear');
        else
            interpForce(j,i) = torque(1,i);
        end
    end
end

torqueVsSpeed = zeros(length(speedArray),3);
torqueVsSpeed(:,1) = speedArray;
%Selecting best gear for each speed
for i = 1:length(speedArray)
    [torqueVsSpeed(i,2),torqueVsSpeed(i,3)] = max(interpForce(i,:));
end