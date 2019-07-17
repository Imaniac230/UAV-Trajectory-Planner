function [varargout] = trjterrfilt(InMat,TerrMod,varargin)
%TRJTERRFILT - Calculate closest distances from terrain or filter unwanted close points
%
%   This function calculates distances between trajectory waypoints [InMat] and closest points
%   from the terrain [TerrMod]. If the optional third parameter [RDist] is provided, the function
%   filters out all points that are closer than [RDist].
%
%   !Output parameters depend on the mode of the function. If only the distances are calculated,
%   only two outputs are given: a vector of distances [MinDist] and an average minimal distance [avgMinDist].
%   If the optional third parameter [RDist] is provided, a filtered matrix of waypoints [OutMat] is additionaly
%   given as the first paramter.!
%
%   [MinDist,avgMinDist] = TRJTERRFILT(InMat,TerrMod)
%   [OutMat,MinDist,avgMinDist] = TRJTERRFILT(InMat,TerrMod,RDist)
%
%   The input trajectory [InMat] and digital elevation model [TerrMod] must be matrices of geodetic (polar) coordinates
%   with 3 columns [latitude longitude height] respectively. Distance [RDist] must be a non-zero positive 
%   value given in meters [m].

%%
%
TerrModRes = demresol(TerrMod,10); %[m]
%%
%inicializacne parametre %initial parameters
errMat = 'Invalid input trajectory (1st parameter). Input must be a matrix of polar coordinates with 3 columns. For more info please visit help.';
errDist = 'Invalid radial distance (3rd parameter). Input must be a non-zero positive value. For more info please visit help.';
errModel = 'Invalid terrain elevation model (2nd parameter). Input must be a matrix of polar coordinates with 3 columns. For more info please visit help.';
errFilt = 'Too many output arguments. If you want to filter waypoints please specify the minimum distance. For more info please visit help.';
if ((size(InMat,2) ~= 3) || ~isnumeric(InMat))
    error(errMat)
end
if ((size(TerrMod,2) ~= 3) || ~isnumeric(TerrMod))
    error(errModel)
end
if (size(varargin,2) > 1)
    error('Too many input arguments.')
end
if (nargin == 3)
    if (size(varargin{1},1) > 1 || size(varargin{1},2) > 1 || varargin{1} <= 0)
        error(errDist)
    end
    RDist = varargin{1} + TerrModRes;
end

OutMat = zeros(size(InMat));
%%
%prevod z polarnej na lokalnu sustavu %conversion from polar to local frame
TerrModL = zeros(size(TerrMod));
InMatL = zeros(size(InMat));
[TerrModL(:,1),TerrModL(:,2),TerrModL(:,3)] = geodetic2enu(TerrMod(:,1),TerrMod(:,2),TerrMod(:,3),InMat(1,1),InMat(1,2),InMat(1,3),wgs84Ellipsoid);
[InMatL(:,1),InMatL(:,2),InMatL(:,3)] = geodetic2enu(InMat(:,1),InMat(:,2),InMat(:,3),InMat(1,1),InMat(1,2),InMat(1,3),wgs84Ellipsoid);
%%
%volitelne odfiltrovanie waypointov %optional waypoint filtering
if (nargin == 3)
    TrjIdx = 1;
    for i = 1:size(InMatL,1)
        RangeHor = sqrt((TerrModL(:,1) - InMatL(i,1)).^2 + (TerrModL(:,2) - InMatL(i,2)).^2);
        TerrPoints = TerrModL(RangeHor<=RDist,:);

        TrjModDdist = sqrt((TerrPoints(:,1) - InMatL(i,1)).^2 + (TerrPoints(:,2) - InMatL(i,2)).^2 + (TerrPoints(:,3) - InMatL(i,3)).^2);
        TrjModDistMin = min(TrjModDdist(TrjModDdist<(RDist - TerrModRes),:));

        if (isempty(TrjModDistMin))
            OutMat(TrjIdx,:) = InMat(i,:);
            TrjIdx = TrjIdx + 1;
        end
    end
    OutMat(~any(OutMat,2),:) = [];
    %vypocet vzdialenosti %distance calculations
    OutMatL = zeros(size(OutMat));
    [OutMatL(:,1),OutMatL(:,2),OutMatL(:,3)] = geodetic2enu(OutMat(:,1),OutMat(:,2),OutMat(:,3),InMat(1,1),InMat(1,2),InMat(1,3),wgs84Ellipsoid);
    if (isempty(OutMat))
        MinDist = NaN;
        avgMinDist = NaN;
    else
        MinDist = zeros(size(InMat,1),1);
        for i = 1:size(OutMatL,1)
            TrjModDist = sqrt((TerrModL(:,1) - OutMatL(i,1)).^2 + (TerrModL(:,2) - OutMatL(i,2)).^2 + (TerrModL(:,3) - OutMatL(i,3)).^2);
            MinDist(i) = min(TrjModDist);
        end
        MinDist(~any(MinDist,2),:) = [];
        avgMinDist = sum(MinDist)/size(MinDist,1);
    end
    %vystupne parametre %output parameters
    if (nargout > 3)
        error('Too many output arguments.')
    end
    varargout{1} = OutMat;
    varargout{2} = MinDist;
    varargout{3} = avgMinDist;
else
%%
%samotny vypocet vzdialenosti %only distance calculations
    MinDist = zeros(size(InMat,1),1);
    for i = 1:size(InMatL,1)
        TrjModDist = sqrt((TerrModL(:,1) - InMatL(i,1)).^2 + (TerrModL(:,2) - InMatL(i,2)).^2 + (TerrModL(:,3) - InMatL(i,3)).^2);
        MinDist(i) = min(TrjModDist);
    end
    avgMinDist = sum(MinDist)/size(MinDist,1);
    %vystupne parametre %output parameters
    if (nargout > 2)
        error(errFilt)
    end
    varargout{1} = MinDist;
    varargout{2} = avgMinDist;
end
end