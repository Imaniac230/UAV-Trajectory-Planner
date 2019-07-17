function [TerrModRes] = demresol(TerrMod,NofPoints)
%DEMRESOL Calculate the resolution of a digital elevation model in meters
%   This function calculates the spacial resolution of the input digital elevation model [TerrMod].
%   [NofPoints] defines the number of points from the model used to calculate the resolution. The moints used,
%   the greater the precision but at the cost of slower performance. Please use this parameter cautiously.
%   Output is a value representing the resolution in meters [m]. 
%
%   [TerrModRes] = DEMRESOL(TerrMod,NofPoints)
%
%   The input elevation model [TerrMod] must be a matrix of geodetic (polar) coordinates
%   with 3 columns [latitude longitude height] respectively. 
%   [NofPoints] must be a single integer defining the number to be used for calculations.

%%
%
%%
%inicializacne parametre %initial parameters
errMod = 'Invalid input model (1st parameter). Input must be a matrix of polar coordinates with 3 columns. For more info please visit help.';
errPoint = 'Invalid number of points (2nd parameter). Input must be a single integer value. For more info please visit help.';
if (~isnumeric(TerrMod) || (size(TerrMod,2) ~= 3))
    error(errMod)
end
if (~isnumeric(NofPoints) || (sum(size(NofPoints)) ~= 2) || (mod(NofPoints,1) ~= 0))
    error(errPoint)
end
ccount = sum(TerrMod(:,1) < 0) + sum(TerrMod(:,2) < 0);
if (ccount)
    warning('DEMResolution:BadData','%d input coordinates have negative values! Coordinates are either corrupt or not in geodetic (polar) form by wgs84 standard.',ccount)
end
%%
%prevod z polarnej na lokalnu sustavu %conversion from polar to local frame
TerrModL = zeros(size(TerrMod,1),2);
[TerrModL(:,1),TerrModL(:,2)] = geodetic2enu(TerrMod(:,1),TerrMod(:,2),0,TerrMod(1,1),TerrMod(1,2),0,wgs84Ellipsoid);
%%
%urcenie rozlisenia %resolution finding
Dists = zeros(NofPoints,1);
for i = 1:NofPoints
    PointDists = sqrt((TerrModL(i,1) - TerrModL(:,1)).^2 + (TerrModL(i,2) - TerrModL(:,2)).^2);
    PointDists(PointDists == 0) = [];
    Dists(i) = min(PointDists);
end

TerrModRes = sum(Dists)/size(Dists,1);
end