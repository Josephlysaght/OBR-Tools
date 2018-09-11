minCurvature = -0.27;
maxCurvature = 0.275;

speedSweepReduction = 0.0;

tic
numSweepPoints = 200;
maxSpeedTolerance = 0.1;
topSpeed = max(Car.Powertrain.torqueVsSpeed(:,1));
curvSweep = linspace(minCurvature,maxCurvature,numSweepPoints)';
speedSweep = zeros(length(curvSweep),1);
accelXSweep = zeros(length(curvSweep),1);
stateVarSweep = zeros(length(curvSweep),13);
lastSpeed = 0;

for i= 1:length(curvSweep)
    % Use binary search to find limit speed
    
    % base initial guess from previous speed
    [testStateVar,testAccelX]=solver(Car,lastSpeed,curvSweep(i),0);
    
    if isnan(testAccelX)
        % If solver failed than testSpeed is too high and should be
        % assigned to upper bound
        upperSpeed = lastSpeed;
        speedSweep(i) = 0;
    else
        speedSweep(i) = lastSpeed;
        accelXSweep(i) = testAccelX;
        stateVarSweep(i,:) = testStateVar;
        upperSpeed = topSpeed;
    end
    
    if upperSpeed<speedSweep(i)
        %If upperSpeed is not greater than switch them around
        savedSpeed = upperSpeed;
        upperSpeed = speedSweep(i);
        speedSweep(i) = savedSpeed;
        clear savedSpeed
    end
    
    % iterate binary search until below tolerance
    while upperSpeed-speedSweep(i)>maxSpeedTolerance

        testSpeed = (upperSpeed+speedSweep(i))/2;

        [testStateVar,testAccelX]=solver(Car,testSpeed,curvSweep(i),0);
        
        if isnan(testAccelX)
            % If solver failed than testSpeed is too high and should be
            % assigned to upper bound
            upperSpeed = testSpeed;
        else
            speedSweep(i) = testSpeed;
            accelXSweep(i) = testAccelX;
            stateVarSweep(i,:)=testStateVar;
        end
        if upperSpeed<speedSweep(i)
            %If upperSpeed is not greater than switch them around
            savedSpeed = upperSpeed;
            upperSpeed = speedSweep(i);
            speedSweep(i) = savedSpeed;
            clear savedSpeed
        end
            
    end
    if speedSweep(i) == 0
         warning(strcat('The max speed is zero for curvature of',num2str(Track.curvature(i)),'. Either corner is impossible for car or solver unable to converge for that point.'))
    end
    
    lastSpeed = speedSweep(i);
end

toc
%Reduce speed sweep slightly to account for errors when interpolating that
%would cause the simulation to get car to a speed that cannot be solved
speedSweep = speedSweep-speedSweepReduction;

numReducedPoints = 50;
downsampledCurv = downsample(curvSweep,ceil(numSweepPoints/numReducedPoints));
downsampledSpeed = downsample(speedSweep,ceil(numSweepPoints/numReducedPoints));

testCurvatures = linspace(minCurvature,maxCurvature,numSweepPoints);


for i = 1:length(testCurvatures)
    testspeed(i) = interp1(downsampledCurv,downsampledSpeed,testCurvatures(i),'pchip');
%     [teststateVar, testaccelX] = solver(Car, testspeed(i), testCurvatures(i), 0);
%     if isnan(testaccelX)
%         pass(i) = 0;
%     else
%         pass(i) = 1;
%     end
end


figure
hold on
plot(curvSweep,speedSweep)
plot(testCurvatures,testspeed,'*')
