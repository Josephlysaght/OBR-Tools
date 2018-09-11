function [maxSpeed, stateVar, accelX] = findMaxSpeed(Car, curvature, tolerance)
% *************************************************************************
% FUNCTION NAME:
%   findMaxSpeed
%
% DESCRIPTION:
%   Use binary search to find limit speed
%
% INPUTS:
%   Car - car data struct
%   curvature - (1/m)
%   tolerance - search stopping criteria. Accuracy of search (m/s)
%
% OUTPUTS:
%   maxSpeed - maximum speed possible (m/s)
%   stateVar - vehicle state at max speed
%   accelX - longitudinal acceleration at max speed (m/s)
% 
% KNOW ISSUES:
%
% CHANGE LOG:
%   2018-04-02: Initial revision
% *************************************************************************


maxSpeed = 0;
accelX = nan();
stateVar = nan();
upperSpeed = max(Car.Powertrain.torqueVsSpeed(:,1));

%try top speed
[testAccelX, testStateVar]=solver(Car,upperSpeed,curvature,0);
if ~isnan(testAccelX)
    maxSpeed = upperSpeed;
    accelX = testAccelX;
    stateVar = testStateVar; 
end

% iterate binary search until below tolerance
while upperSpeed-maxSpeed>tolerance
    
    testSpeed = (upperSpeed+maxSpeed)/2;
    
    [testAccelX, testStateVar]=solver(Car,testSpeed,curvature,0);
    
    if isnan(testAccelX)
        % If solver failed than testSpeed is too high and should be
        % assigned to upper bound
        upperSpeed = testSpeed;
    else
        maxSpeed = testSpeed;
        accelX = testAccelX;
        stateVar = testStateVar;
    end
    if upperSpeed<maxSpeed
        %If upperSpeed is not greater than switch them around
        savedSpeed = upperSpeed;
        upperSpeed = maxSpeed;
        maxSpeed = savedSpeed;
        clear savedSpeed
    end
    
end
if maxSpeed == 0
    warning(strcat('The max speed is zero for curvature of', 32, num2str(curvature),'. Either corner is impossible for car or solver unable to converge for that point.'))
end
end