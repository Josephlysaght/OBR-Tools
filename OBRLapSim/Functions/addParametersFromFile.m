function [returnStruct] = addParametersFromFile(inputStruct, filename)
% *************************************************************************
% FUNCTION NAME:
%   addParametersFromFile
%
% DESCRIPTION:
%   Read data out of excel file and adds variables to inputStruct structure
%   Variable name should be in first column and values in second column.
%   First row in spreadsheet is a header row.
%
% INPUTS:
%   inputStruct - struct that the data from teh file will be added to
%   filename -  string that is the file name or path
%
% OUTPUTS:
%   outputStruct - return the input struct with the data from the file
% 
% KNOW ISSUES:
%
% CHANGE LOG:
%   2018-04-02: Initial revision
% *************************************************************************



returnStruct=inputStruct;

[num,txt]=xlsread(filename);
txt(1,:)=[];

for n=1:length(num)
    if ~(isnan(num(n)))
        returnStruct.(txt{n,1})=num(n,1);
    else
        returnStruct.(txt{n,1})=txt(n,2);
    end
end
