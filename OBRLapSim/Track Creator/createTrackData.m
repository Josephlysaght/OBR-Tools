function [TrackData] = createTrackData(Time,LatAcc,Velocity,closedTrack)
%This script turns lateral acceleration, speed and time into X and Y
%coordinates for a certain track. You may need to change de name of the
%channels depending on the method you use to calculate yaw rate.

% Make closed track true if it is a circuit (eg. endurance) and false for
% autocross tyre tracks that have a separate beginning and end point


LatAcc = (9.81*LatAcc);
Velocity = Velocity/3.6;

curvature = LatAcc./Velocity.^2;

%Deriving yaw rate
Yawrate = LatAcc./Velocity;
%Finding heading angle
yawpos = cumtrapz(Time,Yawrate);
%Scaling
yawpos = 2*pi*yawpos/yawpos(end);
d = cumtrapz(Time,Velocity);

X = zeros(length(Time),1);
X(1) = 0;
Y = zeros(length(Time),1);
Y(1) = 0;


for n=2:length(Time)
    
    X(n) = X(n-1)+cos(yawpos(n))*(d(n)-d(n-1));
    Y(n) = Y(n-1)+sin(yawpos(n))*(d(n)-d(n-1));

end

if closedTrack
    %Link ends of track map
    %Finding distance gap between initial and final point
    %and spliting equally between points of the track
    deltaX = (X(1) - X(end))/length(X);
    deltaY = (Y(1) - Y(end))/length(X);

    %Finding new track coordinates
    for n=2:length(Time)
        X(n) = X(n-1) + cos(yawpos(n))*(d(n) - d(n-1))...
            + deltaX;
        Y(n) = Y(n-1) + sin(yawpos(n))*(d(n) - d(n-1))...
            + deltaY;
    end
end

TrackData = struct();
TrackData.d = d;
TrackData.X = X;
TrackData.Y = Y;
TrackData.curvature = curvature;