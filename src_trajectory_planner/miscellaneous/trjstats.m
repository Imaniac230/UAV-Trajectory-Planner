function [PointDist,avgHeight,avgPDist,avgLDist,LCount] = trjstats(InMat)
%TRJSTATS Statistics of the trajectory matrix
%   
%   This function provides basic information about the input trajectory matrix [InMat].
%   Outputs are: average distance between trajectory lines [avgLDist] and their number [LCount]
%   for the generated trajectory, average distance between waypoints [avgPDist]
%   for the segmented trajectory, average terrain height [avgHeight] for the mapped trajectory
%   and a vector of individual distances between consecutive trajectory points [PointDist].
%
%   [PointDist,avgHeight,avgPDist,avgLDist,LCount] = TRJSTATS(InMat)
%
%   Input must be a matrix of geodetic (polar) coordinates with 2 or 3 columns
%   representing [latitude longitude height] respectively.

%%
%inicializacne parametre %initial parameters
errMat = 'Invalid input parameter. Input must be a matrix of polar coordinates with 2 or 3 columns. For more info please visit help.';
if ((size(InMat,2) ~= 2) && (size(InMat,2) ~= 3))
    error(errMat)
end
ccount = sum(InMat(:,1) < 0) + sum(InMat(:,2) < 0);
if (ccount)
    warning('TrajectoryStats:BadData','%d input coordinates have negative values! Coordinates are either corrupt or not in geodetic (polar) form by wgs84 standard.',ccount)
end
%%
%prevod z polarnej na lokalnu sustavu %conversion from polar to local frame
InMatL = InMat;
[InMatL(:,1),InMatL(:,2)] = geodetic2enu(InMat(:,1),InMat(:,2),0,InMat(1,1),InMat(1,2),0,wgs84Ellipsoid);
%%
%vzdialenost medzi po sebe nasledujucimi bodmi trajektorie
%distance between consecutive trajectory points
PointDist = zeros(size(InMat,1)-1,1);
for i = 1:(size(InMatL,1) - 1)
    PointDist(i) = sqrt((InMatL(i,1)-InMatL(i+1,1))^2 + (InMatL(i,2)-InMatL(i+1,2))^2);
end
%priemerna vzdialenost medzi po sebe nasledujucimi bodmi trajektorie
%average distance between consecutive trajectory points
avgPDist = sum(PointDist)/size(PointDist,1);
%%
%pocet ciar trajektorie %number of trajectory lines
angleR = atan(InMatL(2,2)/InMatL(2,1));
R = [cos(-angleR) -sin(-angleR) 0; sin(-angleR) cos(-angleR) 0; 0 0 1];
InMatLtmp = InMatL';
if (size(InMatLtmp,1) ~= 3)
    InMatLtmp(3,:) = 0;
end
InMatL = (R*InMatLtmp)';
if ((size(InMatL,1) > 2) && (InMatL(3,2) < 0 && InMatL(4,2) < 0))
    angleR = angleR + pi;
    R = [cos(-angleR) -sin(-angleR) 0; sin(-angleR) cos(-angleR) 0; 0 0 1];
    InMatL = (R*InMatLtmp)';
end
LMat = zeros(size(InMat));
%LMat(1,1:2) = InMatL(1,1:2);
inv = false;
for i = 2:size(InMat,1)-1
    if ((abs(InMatL(i,2) - InMatL(i+1,2)) > 0.000001) && ~inv)
        LMat(i,1:2) = InMatL(i,1:2);
        inv = true;
    end
    if ((abs(InMatL(i,2) - InMatL(i+1,2)) < 0.000001) && inv)
        LMat(i,1:2) = InMatL(i,1:2);
        inv = false;
    end
end
LMat(end,1:2) = InMatL(end,1:2);
LMat(1,:) = 1;
LMat(~any(LMat,2),:) = [];
LMat(1,:) = 0;
LCount = ceil(size(LMat,1)/2);
%priemerna vzdialenost medzi ciarami trajektorie
%average distance between trajectory lines
avgLDist = 0;
for i = 1:(LCount - 1)
    %avgLDist = avgLDist + sqrt(PointDist(2*i)^2 - (InMatL(2*i+1,1)-InMatL(2*i,1))^2);
    avgLDist = avgLDist + abs(LMat(2*i+1,2) - LMat(2*i,2));
end
avgLDist = avgLDist/(LCount - 1);
%%
%priemerma vyska trajektorie %average trajectory height
if (size(InMat,2) == 3)
    avgHeight = sum(InMat(:,3));
    ccount = sum(InMat(:,3) < 0);
    if (ccount)
        warning('TrajectoryStats:BadData','%d corrupt height coordinates with negative values!',ccount)
    end
    avgHeight = avgHeight/size(InMat,1);
else
    avgHeight = NaN;
end