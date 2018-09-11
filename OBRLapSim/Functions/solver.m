function [accelX, stateVar] = solver(Car, speed, curvature, throttleOrBrake)
% *************************************************************************
% FUNCTION NAME:
%   solver
%
% DESCRIPTION:
%   Newton Raphson based solver for static state of vehicle model. 
%   Damping is added to prevent solution from diverging
%
% INPUTS:
%   Car - car data struct
%   speed - (m/s)
%   curvature - (1/m)
%   throttleOrBrake - throttle (%) or brake force (N)
%
% OUTPUTS:
%   accelX - longitudinal acceleration (m/s^2). Returns NaN if solver is not converged
%   stateVar - car state satisfying steady state condition
% 
% KNOW ISSUES:
%
% CHANGE LOG:
%   2018-04-02: Initial revision
%   2018-04-03: Reduced maxFirstCurvature and increased
%               curvatureStepReductionOrder to solve convergence issue for
%               vehicle model without aero
% *************************************************************************


numState = 17; % number of state variables
numObj = numState; % number of objectives (has to be the same as numState)
errorTol = 1;
% Specify number of iterations
if speed<25
    maxIter = 10;
else
    maxIter = 20;
end
% Specify damping
dampingFactor = 0.2;
dampingOrder = 2;

% Determine number of curavtures to step up with
maxFirstCurvature = 0.08; % increase if curvatures below this value are failing to converge
minNumCurvature = 1;
curvatureStepReductionOrder = 0.9; % increase if curvatures beyond maxFirstCurvature are failing to converge

numCurvature = ceil((abs(curvature)/maxFirstCurvature)^curvatureStepReductionOrder);
if numCurvature < minNumCurvature
    numCurvature = minNumCurvature;
end

% If speed is zero make it a very small number to avoid divding by zero
if speed == 0
    speed = 0.1;
end

J = zeros(numObj,numState);

curvaturePoints = zeros(1,numCurvature);
for j=1:numCurvature
    curvaturePoints(j) = (j/numCurvature)^(1/curvatureStepReductionOrder)*curvature;
end

% Create initial guess
x = zeros(numState,1);
   
for c = curvaturePoints
    % Refine guess for specific curvature. These are conditions that are very sensitive to curvature
    % Steering angle is ackerman angle
    x(1) = Car.Chassis.wheelbase*c*180/pi/Car.Kin.steeringRate;
    % Wheel angular velocities match forward wheel speed to get zero slip ratio
    x(10) = (speed-speed*c*Car.Chassis.centerlineFL)/Car.Chassis.tyreRadius;
    x(11) = (speed+speed*c*Car.Chassis.centerlineFR)/Car.Chassis.tyreRadius;
    x(12) = (speed-speed*c*Car.Chassis.centerlineRL)/Car.Chassis.tyreRadius;
    x(13) = (speed+speed*c*Car.Chassis.centerlineRR)/Car.Chassis.tyreRadius;
    
    
    [stateDeriv,accelX] = vehicleModel(Car,x,speed,c,throttleOrBrake);
    f=stateDeriv-stateDerivTarget(Car,speed,c,accelX);
    error = sum(f.^2)/numObj;
    count = 0;

    while (error>errorTol) && (count<maxIter)
        x_prev = x;
        
        % Create Jacobian
        for n=1:numState
            h = zeros(numState,1);
            h(n) = sign(x_prev(n)+0.001)*0.001; %step direction depending on whether positive or negtive
            f_xPlusH = vehicleModel(Car,(x+h),speed,c,throttleOrBrake)-stateDerivTarget(Car,speed,c,accelX);
            J(:,n) = (f_xPlusH-f)./(h(n));
        end
        
        deltaX = J\f;
        damping = 1/(1+dampingFactor*sum(abs(deltaX)/numState)^dampingOrder);
        x = x_prev-damping*deltaX;
        if abs(x(1))>Car.Chassis.maxSteering
            x(1) = sign(x(1))*Car.Chassis.maxSteering;
        end
        [stateDeriv,accelX] = vehicleModel(Car,x,speed,c,throttleOrBrake);
        f = stateDeriv-stateDerivTarget(Car,speed,c,accelX);
        error_prev = error;
        error = sum(f.^2)/numObj;
        
        count = count + 1;
    end

    if error>errorTol
        break
    end

end

% check if converged and return NaN if not
if error>errorTol || isnan(error)
    accelX = nan();
end
stateVar = x;
end