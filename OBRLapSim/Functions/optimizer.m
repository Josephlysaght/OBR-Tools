function [accelX,throttleOrBrake,stateVar] = optimizer(Car,speed,curvature,mode,prevThrottleorBrake,maxAccelX)
% *************************************************************************
% FUNCTION NAME:
%   optimizer
%
% DESCRIPTION:
%   Optimzes throttle or braking for maximum possible Ax
%       Modes:
%       0 - Coast no throttle or brake
%       1 - maximize acceleration staying below maxAccelX
%       2 - maxmize braking staying above maxAccelX
%       3 - maintain speed
%
% INPUTS:
%   Car - car data struct
%   speed - (m/s)
%   curvature - (1/m)
%   mode - optimizer mode (see description)
%   prevThrottleorBrake - starting value for throttle or brake guess
%   maxAccelX - Limit acceleration. Optimizer will not exceed this value
%
% OUTPUTS:
%   accelX - longitudinal acceleration at optimal point (m/s^2)
%   throttleOrBrake - throttle of brake value at optimal point (% for throttle, N for brake)
%   stateVar - vehicle state at optimal point
% 
% KNOW ISSUES:
%
% CHANGE LOG:
%   2018-04-02: Initial revision
% *************************************************************************



largeThrottleStep = 0.1;
smallThrottleStep = 0.01;
largeBrakeStep = -100;
smallBrakeStep = -10;

maintainSpeedAccelTol = 0.1;

guessReduction = 0.75;

stepDir = 1; %initialize step direction as positive

% Run code depending on mode selected
switch mode
    case 0
        % No longitudinal input so just solve with 0 for throttle/brake input.
        % No optimization is necessary
        [accelX, stateVar]=solver(Car,speed,curvature,0);
        throttleOrBrake = 0;
        
        
    case 1 %ACCEL
        %Optimize for max acceleration
        %Try previous throttle as starting point
        if prevThrottleorBrake>0
            throttle = prevThrottleorBrake;
        else
            throttle = 0;
        end
        [accelX, stateVar]=solver(Car,speed,curvature,throttle);
        
        %If failed or if greater than maximum accelX allowable try to reduce slightly
        if throttle>0 && (isnan(accelX) || accelX>maxAccelX)
            throttle = ceil(guessReduction*throttle/smallThrottleStep)*smallThrottleStep; %using ceil keep as integer number throttleStep
            [accelX, stateVar]=solver(Car,speed,curvature,throttle);
        end
        
        %If slightly reduced fails start from 0
        if throttle>0 && (isnan(accelX) || accelX>maxAccelX)
            throttle = 0;
            [accelX, stateVar]=solver(Car,speed,curvature,throttle);
        end
        
        iter = 1;
        while throttle<1
            newThrottle = throttle + stepDir*largeThrottleStep;
            if newThrottle>1
                newThrottle = 1;
            elseif newThrottle<0
                newThrottle = 0;
            end
            
            [newAccelX, newStateVar]=solver(Car,speed,curvature,newThrottle);
            
            if newAccelX>accelX
                if newAccelX<maxAccelX
                    throttle = newThrottle;
                    accelX = newAccelX;
                    stateVar = newStateVar;
                else
                    break
                end
            else
                %If first iteration results in decreasing accel, check that two steps
                %back is also lower to ensure that the peak is found in case
                %the starting guess was just past the peak
                if stepDir>0  && throttle>=largeThrottleStep && iter == 1
                    checkThrottle = throttle-largeThrottleStep;
                    [checkAccelX, checkStateVar]=solver(Car,speed,curvature,checkThrottle);
                    if checkAccelX>accelX
                        throttle = checkThrottle;
                        accelX = checkAccelX;
                        stateVar = checkStateVar;
                        stepDir = -1;
                    else
                        break
                    end
                else
                    break
                end
            end
            iter = iter+1;
        end
        stepDir = 1;
        while throttle<1
            newThrottle = throttle + stepDir*smallThrottleStep;
            if newThrottle>1
                newThrottle = 1;
            end
            
            [newAccelX, newStateVar]=solver(Car,speed,curvature,newThrottle);
            
            if newAccelX>accelX
                if newAccelX<maxAccelX
                    throttle = newThrottle;
                    accelX = newAccelX;
                    stateVar = newStateVar;
                else
                    break
                end
            else
                break
            end
        end
        throttleOrBrake = throttle;
        
        
    case 2 %BRAKING
        %Optimize for max deceleration (min accel)
        %Try previous brake as initial guess
        if prevThrottleorBrake<0
            brake = prevThrottleorBrake;
        else
            brake = 0;
        end
        [accelX,stateVar]=solver(Car,speed,curvature,brake);
        %If failed try to reduce slightly
        if isnan(accelX) || accelX<maxAccelX
            brake = floor(guessReduction*brake/largeBrakeStep)*largeBrakeStep; %using ceil keep as integer number throttleStep
            [accelX,stateVar]=solver(Car,speed,curvature,brake);
        end
        %If still failed start at 0
        if isnan(accelX) || accelX<maxAccelX
            brake = 0;
            [accelX,stateVar]=solver(Car,speed,curvature,brake);
        end
        iter = 1;
        
        if accelX>maxAccelX
            while true
                newBrake = brake + stepDir*largeBrakeStep;
                
                [newAccelX,newStateVar]=solver(Car,speed,curvature,newBrake);
                
                if newAccelX<accelX
                    if newAccelX>maxAccelX
                        brake = newBrake;
                        accelX = newAccelX;
                        stateVar = newStateVar;
                    else
                        break
                    end
                else
                    %If first iteration results in increasing accel, check that two steps
                    %back is also lower to ensure that the peak is found in case
                    %the starting guess was just past the peak
                    if stepDir>0  && brake<=largeBrakeStep && iter == 1
                        checkBrake = brake-largeBrakeStep;
                        [checkAccelX, checkStateVar]=solver(Car,speed,curvature,checkBrake);
                        if checkAccelX<accelX
                            brake = checkBrake;
                            accelX = checkAccelX;
                            stateVar = checkStateVar;
                            stepDir = -1;
                        else
                            break
                        end
                    else
                        break
                    end
                end
                iter = iter + 1;
            end
            
            stepDir = 1;
            while true
                newBrake = brake + stepDir*smallBrakeStep;
                
                [newAccelX,newStateVar]=solver(Car,speed,curvature,newBrake);
                
                if newAccelX<accelX
                    if newAccelX>maxAccelX
                        brake = newBrake;
                        accelX = newAccelX;
                        stateVar = newStateVar;
                    else
                        break
                    end
                else
                    break
                end
            end
            throttleOrBrake = brake;
        else
            %If the accel is less than max accel with no braking some throttle is necessary
            throttle = 0;
            while accelX<maxAccelX && throttle<1
                throttle = throttle + smallThrottleStep;
                if throttle>1
                    throttle = 1;
                end
                [accelX,stateVar]=solver(Car,speed,curvature,throttle);
            end
            throttleOrBrake = throttle;
        end
        
        
    case 3 %MAINTAIN SPEED
        throttle = prevThrottleorBrake;
        [accelX, stateVar]=solver(Car,speed,curvature,throttle);
        
        if isnan(accelX)
            throttle = 0;
            [accelX, stateVar]=solver(Car,speed,curvature,throttle);
        end
        
        while true
            %Simulate new throttle
            slope = solver(Car,speed,curvature,throttle+0.01)-accelX;
            newThrottle = throttle-sign(accelX)*sign(slope)*smallThrottleStep;
            if newThrottle < 0
                newThrottle = 0;
            elseif newThrottle > 1
                newThrottle = 1;
            end
            [newAccelX,newStateVar]=solver(Car,speed,curvature,newThrottle);
            
            if abs(newAccelX) < abs(accelX)
                accelX = newAccelX;
                throttle = newThrottle;
                stateVar = newStateVar;
            else
                break
            end
        end
        throttleOrBrake = throttle;
        if abs(accelX)>maintainSpeedAccelTol
            accelX = nan();
            throttleOrBrake = nan();
        end
    otherwise
        accelX = nan();
        throttleOrBrake = nan();
        stateVar = nan();

end

end



