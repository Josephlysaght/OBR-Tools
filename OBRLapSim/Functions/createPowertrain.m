function [Struct] = createPowertrain(Struct,powertrainParametersFile)
% *************************************************************************
% FUNCTION NAME:
%   createPowertrain
%
% DESCRIPTION:
%   Reads powertrain excel file and assigns values
%   Read and store torque curve
%
% INPUTS:
%   Struct - struct to store data to
%   powertrainParameterFile - filename of data file
%
% OUTPUTS:
%   Struct - Struct with added data
% 
% KNOW ISSUES:
%
% CHANGE LOG:
%   2018-04-02: Initial revision
% *************************************************************************


Struct.torqueCurve = xlsread(powertrainParametersFile,'Torque Curve');

[num,txt] = xlsread(powertrainParametersFile,'Drivetrain');
txt(1,:)=[];

for i=1:length(num)
    % if gear ratio value assign to appropriate spot in gear ratio array
    if contains(txt(i,1),'gearRatio')
        gearNum = erase(txt(i,1),'gearRatio'); %get gear number
        Struct.gearRatio(str2double(cell2mat(gearNum))) = num(i,1);
    else
        Struct.(txt{i,1})=num(i,1);
    end
end

end

