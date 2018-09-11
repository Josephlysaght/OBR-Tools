% *************************************************************************
% SCRIPT NAME:
%   LapSim_pureCornering
%
% DESCRIPTION:
%   This script finds and simulates  the maximum curvature for multiple
%   speeds. User specifies the speed range
% 
% KNOW ISSUES:
%
% CHANGE LOG:
%   2018-04-02: Initial revision
%   2018-08-31: Modified to become a function
% *************************************************************************

function Output = LapSim_pureCorneringSweep(Car)

    minSpeed = 8;
    maxSpeed = 25;
    numPoints = 17;
    minCurvature = 0.01;
    curveInc = 0.001;

    Output.speed = linspace(minSpeed,maxSpeed, numPoints)';
    Output.accelX = zeros(length(Output.speed),1);
    Output.throttleOrBrake = zeros(length(Output.speed),1);
    Track.curvature = zeros(length(Output.speed),1);
    testStateVar = 0;
    lastCurvature = minCurvature;

    %% Simulate all speeds starting at high speed

    % Disable some warnings to that show up but can be ignored during the
    % simulation
    warning('off','MATLAB:illConditionedMatrix');
    warning('off','MATLAB:singularMatrix');
    warning('off','MATLAB:nearlySingularMatrix')

    for n=fliplr(1:length(Output.speed))
        %waitbar(((length(Output.speed)-n)-1)/length(Output.speed),h,strcat('Solving:', num2str(Output.speed(n)),'m/s'));
        testCurvature=lastCurvature-curveInc;
        testAccelX = 0;
        count = 1;
        while ~isnan(testAccelX)

            if ~(count == 1)
                %Only record value after first loop
                Track.curvature(n)=testCurvature;
                Output.stateVar(n,:)=testStateVar;
                Output.accelX(n) = testAccelX;
            end

            testCurvature = testCurvature+curveInc;
            [testAccelX, testStateVar]=solver(Car,Output.speed(n),testCurvature,0);

            if count == 1 && isnan(testAccelX)
                % If failed at first attept start curvature at 0
                testAccelX = 0;
                testCurvature = minCurvature-curveInc;
            end
            count = count + 1;
        end

        lastCurvature = Track.curvature(n);
    end

    [Output] = postProcessData(Car,Track,Output);

    % % Enable warnings that were disabled for solving
    % warning('on','MATLAB:illConditionedMatrix');
    % warning('on','MATLAB:singularMatrix');
    % warning('on','MATLAB:nearlySingularMatrix')
    % 
    % close(h)
    % 
    % clearvars -except Car Output Track
    % 
    % % figure
    % subplot(3,2,1)
    % hold on
    % plot(Output.speed,1./Track.curvature)
    % xlabel('Speed (m/s)')
    % ylabel('Corner Radius (m)')
    % 
    % subplot(3,2,2)
    % hold on
    % plot(Output.speed,Output.accelY)
    % xlabel('Speed (m/s)')
    % ylabel('Lateral Acceleration (m/s^2)')
    % 
    % subplot(3,2,3)
    % hold on
    % plot(Output.speed,Output.stability)
    % xlabel('Speed (m/s)')
    % ylabel('Stability Derivative (Nm/deg)')
    % 
    % subplot(3,2,4)
    % hold on
    % plot(Output.speed,Output.control)
    % xlabel('Speed (m/s)')
    % ylabel('Control Derivative (Nm/deg)')
    % 
    % subplot(3,2,5)
    % hold on
    % plot(Output.speed,Output.understeerGradient)
    % xlabel('Speed (m/s)')
    % ylabel('Understeer Gradient (deg/g)')
    % 
    % subplot(3,2,6)
    % hold on
    % plot(Track.curvature,Output.steering)
    % xlabel('Track Curvature (1/m)')
    % ylabel('Steering Angle (deg)')
    %
    % beep
end