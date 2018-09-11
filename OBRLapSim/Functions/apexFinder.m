function [apex] = apexFinder(curvature)
% *************************************************************************
% FUNCTION NAME:
%   apexFinder
%
% DESCRIPTION:
%   This function finds the apexes of all corners on the track.
%   The array Apex is composed by 0 and 1. 1 represents an apex and 0
%   represents another part that is not an apex.
%
% INPUTS:
%   curvature - array of curvature (1/m)
%
% OUTPUTS:
%   apex - array of 0 and 1 indicating apexes
% 
% KNOW ISSUES:
%
% CHANGE LOG:
%   2018-04-02: Initial revision
% *************************************************************************



curvature = abs(curvature);

apex = zeros(length(curvature),1);

for n = 2:length(apex)-1
    
    %Everytime the current corner radius is smaller than the previous one
    %and the next one, it means it is an apex.
    if curvature(n) >= curvature(n-1) && curvature(n) >= curvature(n+1) && curvature(n) > min(curvature)
        apex(n) = 1;
    end

end

% Check if beginning and end points are apexs
if curvature(1) >= curvature(end) && curvature(1) >= curvature(2)
    apex(1) = 1;
end
if curvature(end) >= curvature(end-1) && curvature(end) >= curvature(1)
    apex(end) = 1;
end

end

