% *************************************************************************
% SCRIPT NAME:
%   LapSim_acceleration
%
% DESCRIPTION:
%   This script simulates a 75m meter standing start sprint
% 
% KNOW ISSUES:
%
% CHANGE LOG:
%   2018-04-02: Initial revision
%   2018-08-31: Modified to become a function
% *************************************************************************

function Output = LapSim_acceleration(Car)

    Track = struct();
    Track.d(:,1) = linspace(0,75,76);
    Track.curvature = zeros(length(Track.d),1);
    initSpeed = 0;

    Output = struct();
    Output.time = zeros(length(Track.d),1);
    Output.speed = zeros(length(Track.d),1);
    Output.accelX = zeros(length(Track.d),1);
    Output.throttleOrBrake = zeros(length(Track.d),1);
    Output.stateVar = zeros(length(Track.d),17);

    prevThrottleOrBrake = 0;
    for n = 1:length(Track.d)-1
        [Output.accelX(n),Output.throttleOrBrake(n),stateVar] = optimizer(Car,Output.speed(n),Track.curvature(n),1,prevThrottleOrBrake,100000);
        Output.stateVar(n,:) = stateVar';
        Output.time(n+1) = Output.time(n)+(-Output.speed(n)+sqrt(Output.speed(n)^2+4*0.5*Output.accelX(n)*(Track.d(n+1)-Track.d(n))))/Output.accelX(n);
        Output.speed(n+1) = Output.speed(n)+Output.accelX(n)*(Output.time(n+1)-Output.time(n));
        prevThrottleOrBrake = Output.throttleOrBrake(n);
    end
    
    [Output] = postProcessData(Car,Track,Output);
    %disp(strcat('Acceleration time: ',num2str(max(Output.time)),'s'))
end