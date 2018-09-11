function [Track] = reduceTrackPoints(TrackData,Steps)
% *************************************************************************
% FUNCTION NAME:
%   reduceTrackPoints
%
% DESCRIPTION:
%   This function reduces the number of data points of a track. The idea is
%   just to improve solving time.
%
% INPUTS:
%   TrackData - struct containing original track data
%   Steps - number of points to reduce to
%
% OUTPUTS:
%   Track - struct with reduced track data
% 
% KNOW ISSUES:
%
% CHANGE LOG:
%   2018-04-02: Initial revision
% *************************************************************************


%TrackData: first column is distance. Second is X position. Third is Y position.

%Sample number for original track data
DataNumber = 1:length(TrackData.d);
%Sample number for new track data
SampleNumber = linspace(0,length(TrackData.d),Steps);

%Interpolating original to new data
d(:,1) = interp1(DataNumber,TrackData.d,SampleNumber,'linear','extrap');
Track.d = d - d(1);
X(:,1) = interp1(DataNumber,TrackData.X,SampleNumber,'linear','extrap');
Track.X = X - X(1);
Y(:,1) = interp1(DataNumber,TrackData.Y,SampleNumber,'linear','extrap');
Track.Y = Y - Y(1);
curvature(:,1) = interp1(DataNumber,TrackData.curvature,SampleNumber,'linear','extrap');
Track.curvature = curvature;