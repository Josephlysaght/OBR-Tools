% *************************************************************************
% SCRIPT NAME:
%   LapSim_skidpad
%
% DESCRIPTION:
%   This script simulates skid pad
%   Assumes car is symmetrical and only does left turn
% 
% KNOW ISSUES:
%
% CHANGE LOG:
%   2018-04-02: Initial revision
%   2018-08-31: Modified to become a function
% *************************************************************************

function Output = LapSim_skidpad(Car)
    % Skidpad Simulation

    Track.curvature=1/8.35;
    speedTol = 0.001;
    topSpeed = max(Car.Powertrain.torqueVsSpeed(:,1));

    upperSpeed = topSpeed;
    Output.speed = 0;
    Output.throttleOrBrake = 0;
    while  upperSpeed-Output.speed>speedTol
        % Use binary search to find limit speed
        newSpeed = (upperSpeed+Output.speed)/2;
        [newAccelX,newThrottle,newStateVar] = optimizer(Car,newSpeed,Track.curvature,3,Output.throttleOrBrake);

        if isnan(newAccelX)
            % If solver failed than newSpeed is too high and should be
            % assigned to upper bound
            upperSpeed = newSpeed;
        else
            Output.speed = newSpeed;
            Output.stateVar = newStateVar';
            Output.accelX = newAccelX;
            Output.throttleOrBrake = newThrottle;
        end

        if upperSpeed<Output.speed
            %If upperSpeed is not greater than switch them around
            savedSpeed = upperSpeed;
            upperSpeed = Output.speed;
            Output.speed = savedSpeed;
            clear savedSpeed
        end
    end

    skidpadTime = 2*pi*(1/Track.curvature)/Output.speed;
    Output.time = skidpadTime;

    %disp(strcat('Skidpad time: ',num2str(max(skidpadTime)),'s'))
    [Output] = postProcessData(Car,Track,Output);

    clear topSpeed upperSpeed speedTol newThrottle newSpeed newStateVar newAccelX
end