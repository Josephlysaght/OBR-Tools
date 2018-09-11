function [ans] = AeroInterp(AeroMap, dims, Interp)
% *************************************************************************
% FUNCTION NAME:
%   AeroInterp
%
% DESCRIPTION:
%   Interpolates from aeromap data
%   this function was edited by Darryn Kelbrick beyond original functionality
%
% INPUTS:
%   AeroMap - aeromap data of desired parameter
%   dims - car attitude in vector form [front ride height(mm), rear ride height(mm), yaw(deg), averageSteeredAngle(deg), roll(deg)]
%   Interp - 
%
% OUTPUTS:
%   ans - interpolated value
% 
% KNOW ISSUES:
%
% CHANGE LOG:
%   2018-04-02: Initial revision
% *************************************************************************



dimsMin = [min(dims{1}), min(dims{2}), min(dims{3}), min(dims{4}), min(dims{5})];
dimsMax = [max(dims{1}), max(dims{2}), max(dims{3}), max(dims{4}), max(dims{5})];
interpResults = zeros(length(Interp), 1);
dimsInterp = zeros(5, 2);
diffInterp = zeros(5, 2);
FRH = 1;
RRH = 2;
Y = 3;
S = 4;
R = 5;
% try
%     for i = 1:size(Interp, 1)
%         if(dimsMin<=Interp(i, :) || Interp(i, :)<=dimsMax)
%             for j = 1:5
%                 find(min(abs(dims{j})-Interp(i, j)));
%             end
%         end
%     end
% catch
%     error('Requested interp not within range of aero map. Check dims or rethink Interp');
% end
    for i = 1:size(Interp, 1)
        if(i == 39)
            absd = 1;
        end
        if(dimsMin<=Interp(i, :) & Interp(i, :)<=dimsMax)
            for j = 1:5
                if (~isempty(find(dims{j} == Interp(i, j))))
                    dimsInterp(j, 1) = find(dims{j} == Interp(i, j));
                    dimsInterp(j, 2) = dimsInterp(j, 1);
                    diffInterp(j, 1) = 1;
                    diffInterp(j, 2) = 0;
                else
                    AbsDiff = abs(dims{j} - Interp(i, j));
                    Int = find(AbsDiff == min(AbsDiff), 1);
                    if(Int == 1)
                        LowInt = 1;
                    elseif(Int == length(dims{j}))
                        LowInt = length(dims{j}) - 1;
                    elseif(AbsDiff(Int - 1) >  AbsDiff(Int + 1))
                        LowInt = Int;
                    else
                        LowInt = Int - 1;
                    end
                    dimsInterp(j,:) = [LowInt, LowInt+1];
                    diffInterp(j, 1) = 1 - abs(Interp(i, j) - dims{j}(dimsInterp(j, 1)))/abs(dims{j}(dimsInterp(j, 1))...
                        - dims{j}(dimsInterp(j, 2)));
                    diffInterp(j, 2) = 1 - diffInterp(j, 1);
                end
            end
            lev1{1} = (AeroMap(:,:,dimsInterp(Y, 1), dimsInterp(S, 1), dimsInterp(R, 1))* diffInterp(Y, 1))...
                + (AeroMap(:,:,dimsInterp(Y, 2), dimsInterp(S, 1), dimsInterp(R, 1))* diffInterp(Y, 2));

            lev1{2} = (AeroMap(:,:,dimsInterp(Y, 1), dimsInterp(S, 2), dimsInterp(R, 1))* diffInterp(Y, 1))...
                + (AeroMap(:,:,dimsInterp(Y, 2), dimsInterp(S, 2), dimsInterp(R, 1))* diffInterp(Y, 2));

            lev1{3} = (AeroMap(:,:,dimsInterp(Y, 1), dimsInterp(S, 1), dimsInterp(R, 2))* diffInterp(Y, 1))...
                + (AeroMap(:,:,dimsInterp(Y, 2), dimsInterp(S, 1), dimsInterp(R, 2))* diffInterp(Y, 2));

            lev1{4} = (AeroMap(:,:,dimsInterp(Y, 1), dimsInterp(S, 2), dimsInterp(R, 2))* diffInterp(Y, 1))...
                + (AeroMap(:,:,dimsInterp(Y, 2), dimsInterp(S, 2), dimsInterp(R, 2))* diffInterp(Y, 2));

            lev2{1} = (lev1{1}*diffInterp(S, 1)) + (lev1{2}*diffInterp(S, 2));
            lev2{2} = (lev1{3}*diffInterp(S, 1)) + (lev1{4}*diffInterp(S, 2));

            lev3 = (lev2{1}*diffInterp(R, 1)) + (lev2{2}*diffInterp(R, 2));

            Temp1 = lev3(dimsInterp(FRH, 1), dimsInterp(RRH, 1))*(diffInterp(FRH, 1)*diffInterp(RRH, 1))...
                + lev3(dimsInterp(FRH, 1), dimsInterp(RRH, 2))*(diffInterp(FRH, 1)*diffInterp(RRH, 2))...
                + lev3(dimsInterp(FRH, 2), dimsInterp(RRH, 1))*(diffInterp(FRH, 2)*diffInterp(RRH, 1))...
                + lev3(dimsInterp(FRH, 2), dimsInterp(RRH, 2))*(diffInterp(FRH, 2)*diffInterp(RRH, 2));
            
            interpResults(i, :) = Temp1;
        end
    end
    ans = interpResults(1,1);

end