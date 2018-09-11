function TrackMapPlotter(Track,Data)
% *************************************************************************
% FUNCTION NAME:
%   TrackMapPlotter
%
% DESCRIPTION:
%   Tool for plotting coloured track map based on an output array
%
% INPUTS:
%   Track - struct contain track data
%   Data - Array of data for colour pot and scale
%
% OUTPUTS:
%   
% 
% KNOW ISSUES:
%
% CHANGE LOG:
%   2018-04-02: Initial revision
% *************************************************************************
Z=zeros(size(Track.X));
surface([Track.X';Track.X'],[Track.Y';Track.Y'],[Data';Data'],[Data';Data'],'facecol','no','edgecol','interp','linew',2);
xRange = max(Track.X)-min(Track.X);
yRange = max(Track.Y)-min(Track.Y);
axisRange = max(xRange,yRange);
xMid = (max(Track.X)+min(Track.X))/2;
yMid = (max(Track.Y)+min(Track.Y))/2;
xlim([xMid-axisRange/2,xMid+axisRange/2]);
ylim([yMid-axisRange/2,yMid+axisRange/2]);
colorbar
end

