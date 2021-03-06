% *************************************************************************
% SCRIPT NAME:
%   LapSim_JTurn
%
% DESCRIPTION:
%   This script simulates a user specified J-Turn
% 
% KNOW ISSUES:
%
% CHANGE LOG:
%   2018-04-02: Initial revision
% *************************************************************************
function Output = LapSim_JTurn(Car)
    %% Simulation Set up
    % J-turn Sim
    %% Turn Parameters
    startSpeed = 10; %m/s
    cornerRadius = 8.375; %m
    cornerAngle = 90; %deg
    entryLength = 5; %m
    exitLength = 5; %m
    pointSpacing = 1; %m

    %% Create Track
    peakCurvature = 1/cornerRadius;
    cornerLength = 2*cornerAngle*pi/180*cornerRadius;
    totalLength = entryLength+cornerLength+exitLength;

    Track.d = (0:pointSpacing:totalLength)';
    apexDistance = entryLength + 1/2 * cornerLength;
    if ~max(ismember(Track.d,apexDistance))
        Track.d = [Track.d;apexDistance]; %Make sure to include apex as a point
        Track.d = sort(Track.d);
    end
    if ~(Track.d(end) == totalLength)
        Track.d(end+1) = totalLength;
    end

    Track.curvature = zeros(length(Track.d),1);
    Track.X = zeros(length(Track.d),1);
    Track.Y = zeros(length(Track.d),1);

    for n=1:length(Track.d)
        if Track.d(n)>entryLength && Track.d(n)<(entryLength+cornerLength)
            Track.curvature(n) = 0.5*(1-cos(2*pi*(Track.d(n)-entryLength)/cornerLength))*peakCurvature;
        end
    end

    heading = zeros(length(Track.d),1);

    for n = 2:length(Track.d)
        heading(n) = heading(n-1)+(Track.d(n)-Track.d(n-1))*Track.curvature(n);
        Track.X(n) = Track.X(n-1)+cos(heading(n))*(Track.d(n)-Track.d(n-1));
        Track.Y(n) = Track.Y(n-1)+sin(heading(n))*(Track.d(n)-Track.d(n-1));
    end

    clear heading

    [Track.apex] = apexFinder(Track.curvature);
    Track.apex(1) = 1;
    Track.apex(end) = 0;

    %Create array that is the distance between steps
    Track.dDistance = zeros(length(Track.d),1);
    for n = 2:length(Track.dDistance)
        Track.dDistance(n) = Track.d(n)-Track.d(n-1);
    end

    %% Initialize outputs
    Output = struct();
    Output.time = zeros(length(Track.d),1);
    Output.dTime = zeros(length(Track.d),1);
    Output.speed = zeros(length(Track.d),1);
    Output.accelX = zeros(length(Track.d),1);
    Output.throttleOrBrake = zeros(length(Track.d),1);
    Output.stateVar = zeros(length(Track.d),17);

    %%  Find maximum speed for each point for zero throttle
    % Disable some warnings to that show up but can be ignored during the
    % simulation
    warning('off','MATLAB:illConditionedMatrix');
    warning('off','MATLAB:singularMatrix');
    warning('off','MATLAB:nearlySingularMatrix');

    maxSpeedTolerance = 0.1;
    maxSpeed = zeros(length(Track.d),1);
    % Define limits for range of curvature where the car can do top speed
    posMaxSpeedCurv = 0;
    negMaxSpeedCurv = 0;

    for n= 1:length(Track.d)
        waitbar(n/length(Track.d)*0.4,progressBar,'Calculating maximum speeds');

        if Track.curvature(n)<posMaxSpeedCurv && Track.curvature(n)>negMaxSpeedCurv
            maxSpeed(n) = max(Car.Powertrain.torqueVsSpeed(:,1));
            stateVar = zeros(length(Output.stateVar(1,:)),1);
            accelX = 0;
        else
            [maxSpeed(n),stateVar,accelX]=findMaxSpeed(Car, Track.curvature(n), maxSpeedTolerance);
            if maxSpeed(n) ==  max(Car.Powertrain.torqueVsSpeed(:,1))
                if Track.curvature(n)>0 && Track.curvature(n)>posMaxSpeedCurv
                    posMaxSpeedCurv = Track.curvature(n);
                elseif Track.curvature(n)<0 && Track.curvature(n)<negMaxSpeedCurv
                    negMaxSpeedCurv = Track.curvature(n);
                end
            end
        end
        % If it is an apex point assign values to output
        if Track.apex(n) == 1
            Output.accelX(n) = accelX;
            Output.speed(n) = maxSpeed(n);
            Output.stateVar(n,:) = stateVar';
        end
    end

    %% Simulate Corner

    numSector = nnz(Track.apex); % number of non zero apex
    sectorIndex = find(Track.apex);

    tic
    Output.speed(1) = startSpeed;
    Output.speed(end) = maxSpeed(end);
    for s=1:numSector
        % Simulate acceleration

        waitbar((0.95-0.3)*(2*s-1)/(2*numSector)+0.3, progressBar, strcat('Simulating acceleration in sector',32, num2str(s),32, 'out of',32, num2str(numSector)));

        if s<numSector
            sectorEndIndex = (sectorIndex(s+1)-1);
        else
            sectorEndIndex = length(Track.d);
        end
        sectorStartIndex = sectorIndex(s);
        for n=sectorStartIndex:sectorEndIndex
            if  ~isnan(Output.speed(n))
                if ~(n==length(Track.d))
                    maxAccelX = (maxSpeed(n+1)^2-Output.speed(n)^2)/(2*(Track.dDistance(n+1)));
                else
                    maxAccelX = (maxSpeed(1)^2-Output.speed(n)^2)/(2*(Track.dDistance(1)));
                end

                if n==1
                    [Output.accelX(n),Output.throttleOrBrake(n),stateVar] = optimizer(Car,Output.speed(n),Track.curvature(n),1,0,maxAccelX);
                else
                    [Output.accelX(n),Output.throttleOrBrake(n),stateVar] = optimizer(Car,Output.speed(n),Track.curvature(n),1,Output.throttleOrBrake(n-1),maxAccelX);
                end
                Output.stateVar(n,:) = stateVar';
            end
            if ~(n==length(Track.d))
                if Track.apex(n+1)==1 && Output.speed(n)<Output.speed(n+1)
                    %If speed entering apex is lower than apex speed than point is
                    %not an apex
                    Track.apex(n+1) = 0;
                end
                if ~(Track.apex(n+1)==1)
                    % Only change speed of next point if not apex
                    Output.dTime(n+1) = (-Output.speed(n)+sqrt(Output.speed(n)^2+4*0.5*Output.accelX(n)*(Track.dDistance(n+1))))/Output.accelX(n); %quadratic solution to d=at^2+vt
                    Output.speed(n+1) = sqrt(Output.speed(n)^2+2*Output.accelX(n)*Track.dDistance(n+1));
                end
            end
        end

        % Simulate Braking
        waitbar((0.95-0.3)*(2*s)/(2*numSector)+0.3, progressBar, strcat('Simulating deceleration in sector',32, num2str(s),32, 'out of',32, num2str(numSector)));

        % Calculate speed as braking and moving backwards

        n=sectorEndIndex+1;

        k=0; % k used to keep track of how many sectors back we are simulating into
        while n>sectorStartIndex
            if n>length(Track.d)
                % Since it is not a closed circuit start simulation of last
                % sector braking at the end of track instead of start of next
                % sector
                n=n-1;
                curr = n;
                prev = n-1;
            else
                curr = n;
                prev = n-1;
            end
            if n == sectorEndIndex+1 && Track.apex(curr) == 0
                % if start of next sector is not an apex it means that
                % the acceleration simulation continued accelerating into
                % next sector and braking in current sector is uneccessary
                break
            end
            if isnan(Output.speed(prev)) || Output.speed(prev)>Output.speed(curr)
                %If the speed at the next point (WE ARE GOING BACKWARDS!) is higher
                %than the speed at the current point, it might be necessary to brake
                if Track.apex(prev) == 1
                    %If next point is apex but braking is still required it is no
                    %longer an apex
                    Track.apex(prev) = 0;
                    % Since sector is not an apex because braking is still required continue simulating
                    % braking into previous sector unless we are in first
                    % sector
                    k = k+1;
                    if (s-k)>0
                        sectorStartIndex = sectorIndex(s-k);
                    end

                end
                %Previous braking is 0 for point at end of array
                if n==sectorEndIndex+1
                    lastBrake=0;
                else
                    lastBrake=Output.throttleOrBrake(n);
                end

                maxAccelX = (Output.speed(curr)^2-min([maxSpeed(prev),Output.speed(prev)])^2)/(2*(Track.dDistance(curr))); % the maximum speed at previous point is either max speed for stability or speed acheive during accleration

                [Output.accelX(prev),Output.throttleOrBrake(prev),stateVar] = optimizer(Car,Output.speed(curr),Track.curvature(curr),2,lastBrake,maxAccelX);
                Output.dTime(curr) = (-Output.speed(curr)+sqrt(Output.speed(curr)^2+4*0.5*(-Output.accelX(prev))*(Track.dDistance(curr))))/(-Output.accelX(prev));
                Output.speed(prev) = sqrt(Output.speed(curr)^2-2*Output.accelX(prev)*Track.dDistance(curr));
                Output.stateVar(prev,:) = stateVar';

                if isnan(Output.accelX(prev))
                    %If optimizer fails it means that speed of point i is too high
                    %and it is not possible to apply enough throttle to keep decel
                    %below the limit. In this case we must redetermine the speed at
                    %the current point (i) and resimulate the point after it (i+1)
                    while isnan(Output.accelX(prev))
                        Output.speed(curr) = Output.speed(curr)-maxSpeedTolerance;
                        maxAccelX = (Output.speed(curr)^2-min([maxSpeed(prev),Output.speed(prev)])^2)/(2*(Track.dDistance(curr)));
                        [Output.accelX(prev),Output.throttleOrBrake(prev),stateVar] = optimizer(Car,Output.speed(curr),Track.curvature(curr),2,0,maxAccelX);
                    end
                    Output.dTime(curr) = (-Output.speed(curr)+sqrt(Output.speed(curr)^2+4*0.5*Output.accelX(prev)*(Track.dDistance(curr))))/Output.accelX(prev);
                    Output.speed(prev) = sqrt(Output.speed(curr)^2-2*Output.accelX(prev)*Track.dDistance(curr));
                    Output.stateVar(prev,:) = stateVar';
                    if n<sectorEndIndex+1
                        % if not at the end of sector redo the previous point.
                        % Do this by adding 2 to i so that at next step it is
                        % i+1.
                        n=n+2;
                    end
                end
            end
            n=n-1;
        end
    end
    toc

    clear maxAccelX curr prev maxSpeedTolerance apexDistance cornerAngle cornerLength cornerRadius entryLength exitLength k lastBrake negMaxSpeedCurv posMaxSpeedCurv numSector peakCurvature pointSpacing sectorEndIndex totalLength stateVar s sectorIndex sectorStartIndex startSpeed accelX

    % Enable warnings that were disabled for solving
    warning('on','MATLAB:illConditionedMatrix');
    warning('on','MATLAB:singularMatrix');
    warning('on','MATLAB:nearlySingularMatrix')

    %Calculate lap time
    for n=2:length(Output.dTime)
        Output.time(n) = Output.time(n-1)+Output.dTime(n);
    end
    lapTime = Output.time(end)+Output.dTime(1);

    clear n

    disp(strcat('Lap time: ',num2str(max(lapTime)),'s'))
    [Output] = postProcessData(Car,Track,Output);

    waitbar(1, progressBar, 'Simulation complete');
    close(progressBar)
    clear progressBar

    % Generate comparison graph
    figure
    plot(Track.d,Output.speed)
    title('J-Turn Simulation')
    xlabel('Distance (m)')
    ylabel('Speed (m/s)')
    beep
end