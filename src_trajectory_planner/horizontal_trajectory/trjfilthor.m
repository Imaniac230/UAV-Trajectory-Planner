function [OutTraj] = trjfilthor(InTraj,HorDist)
%TRJFILTHOR - Filter trajectory points in horizontal plane
%
%   This function filters the trajectory points [InTraj] according to the specified minimal horizontal waypoint separation [HorDist]
%
%   [OutTraj] = TRJOFFSETVER(InTraj,HorDist)
%
%   The input trajectory [InTraj] must be a matrix of geodetic (polar) coordinates with 2 or 3 columns
%   [latitude longitude height] respectively. Waypoint separation [HorDist] must be a non-zero positive value given in meters [m].

%%
%inicializacne parametre %initial parameters
errTraj = 'Invalid input trajectory (1st parameter). Input must be a matrix of polar coordinates with 2 or 3 columns. For more info please visit help.';
errDist = 'Invalid minimal waypoint separation (2nd parameter). Input must be a non-zero positive value. For more info please visit help.';
if (HorDist <= 0)
    error(errDist)
end
if (~isnumeric(InTraj) || ((size(InTraj,2) ~= 2) && (size(InTraj,2) ~= 3)))
    error(errTraj)
end
OutTraj = InTraj;
%%
%prevod z polarnej na lokalnu sustavu %conversion from polar to local frame
InTrajL = zeros(size(InTraj,1),2);
[InTrajL(:,1),InTrajL(:,2)] = geodetic2enu(InTraj(:,1),InTraj(:,2),0,InTraj(1,1),InTraj(1,2),0,wgs84Ellipsoid);
%%
%filtrovanie bodov %waypoint filtering
WPSepar = zeros(size(InTrajL,1)-1,1);
for i = 1:(size(InTrajL,1) - 1)
    WPSepar(i) = sqrt((InTrajL(i,1)-InTrajL(i+1,1))^2 + (InTrajL(i,2)-InTrajL(i+1,2))^2);
end
for i = 2:size(OutTraj,1)-1
    if (WPSepar(i) < HorDist)
        OutTraj(i,:) = 0;
    end
end
OutTraj(~any(OutTraj,2),:) = [];
end

