function [OutTraj] = trjradialdistver(InTraj,RDist,TerrMod,varargin)
%TRJRADIALDISTVER - Maintain a minimal radial distance from the terrain
%
%   This function modifies the vertical and horizontal positions of trajectory waypoints [InTraj] to maintain
%   a minimal radial distance [RDist] from all points of the terrain [TerrMod] in all directions. 
%   The position of trajectory waypoints depends on the specified distance and the shape of the terrain.
%   Output is a matrix of geodetic (polar) coordinates with 3 columns representing [latitude longitude height]
%   respectively.
%   The optional minimal distance between waypoints [MinWPSepar] filters out final waypoints that are closer
%   to their next neighbour than the specified distance.
%
%   [OutTraj] = TRJRADIALDISTVER(InTraj,RDist,TerrMod)
%   [OutTraj] = TRJRADIALDISTVER(InTraj,RDist,TerrMod,MinWPSepar)
%
%   The input trajectory [InTraj] and digital elevation model [TerrMod] must be matrices of geodetic (polar) coordinates
%   with 3 columns [latitude longitude height] respectively. Radial distance [RDist] must be a non-zero positive
%   value given in meters [m].
%   The optional minimal distance between waypoints [MinWPSepar] must be a positive value given in meters [m].

%%
%
TerrModRes = demresol(TerrMod,10); %[m]
%%
%inicializacne parametre %initial parameters
errTraj = 'Invalid input trajectory (1st parameter). Input must be a matrix of polar coordinates with 3 columns. For more info please visit help.';
errRdist = 'Invalid radial distance (2nd parameter). Input must be a non-zero positive value. For more info please visit help.';
errModel = 'Invalid elevation model (3rd parameter). Input must be a matrix of polar coordinates with 3 columns. For more info please visit help.';
errMinDist = 'Invalid minimal waypoint separation (last parameter). Input must be a positive value. For more info please visit help.';
if ((size(InTraj,2) ~= 3) || ~isnumeric(InTraj))
    error(errTraj)
end
if (RDist <= 0)
    error(errRdist)
end
if ((size(TerrMod,2) ~= 3) || ~isnumeric(TerrMod))
    error(errModel)
end
if (size(varargin,2) > 1)
    error('Too many input arguments.')
end
if (nargin == 4)
    if (~isnumeric(varargin{1}) || size(varargin{1},1) > 1 || size(varargin{1},2) > 1 || varargin{1} < 0)
        error(errMinDist)
    end
    MinWPSepar = varargin{1};
end

OutTraj = zeros(size(InTraj));
RDist = RDist + TerrModRes;
HorDistLim = RDist;
%%
%pociatocna aplikacia vertikalneho offsetu %initial vertical offset application
InTraj(:,3) = InTraj(:,3) + RDist;
%prevod z polarnej na lokalnu sustavu %conversion from polar to local frame
TerrModL = zeros(size(TerrMod));
InTrajL = zeros(size(InTraj));
[TerrModL(:,1),TerrModL(:,2),TerrModL(:,3)] = geodetic2enu(TerrMod(:,1),TerrMod(:,2),TerrMod(:,3),InTraj(1,1),InTraj(1,2),InTraj(1,3),wgs84Ellipsoid);
[InTrajL(:,1),InTrajL(:,2),InTrajL(:,3)] = geodetic2enu(InTraj(:,1),InTraj(:,2),InTraj(:,3),InTraj(1,1),InTraj(1,2),InTraj(1,3),wgs84Ellipsoid);
%aplikacia korekcie plohy waypointov %waypoint location correction
for j = 1:size(InTrajL,1)
    %overenie vzdialenosti od terenu %distance from terrain verification
    CentPoint = InTrajL(j,:);
    CircRange = sqrt((TerrModL(:,1) - InTrajL(j,1)).^2 + (TerrModL(:,2) - InTrajL(j,2)).^2);
    CircPoints = TerrModL(CircRange<=HorDistLim,:);
    
    CircTrjDist = sqrt((CircPoints(:,1) - InTrajL(j,1)).^2 + (CircPoints(:,2) - InTrajL(j,2)).^2 + (CircPoints(:,3) - InTrajL(j,3)).^2);
    CircTrjMinDist = min(CircTrjDist(CircTrjDist<(RDist - TerrModRes),:));
    if (~isempty(CircTrjMinDist))
        MinDistCoor = CircPoints(CircTrjMinDist == CircTrjDist,:);
    end
    %iterativne korekcie bodu priliz blizko terenu %iterative corrections of point too close to terrain
    while (~isempty(CircTrjMinDist))
        PrevPoint = InTrajL(j,:);
        nfactor = RDist/CircTrjMinDist;
        Zdist = abs(InTrajL(j,3) - MinDistCoor(3));
        Ydist = InTrajL(j,2) - MinDistCoor(2);
        Xdist = InTrajL(j,1) - MinDistCoor(1);
        Zcorrect = Zdist * nfactor;
        Ycorrect = Ydist * nfactor;
        Xcorrect = Xdist * nfactor;
        
        InTrajL(j,3) = MinDistCoor(3) + Zcorrect;
        InTrajL(j,2) = InTrajL(j,2) + (Ycorrect - Ydist);
        InTrajL(j,1) = InTrajL(j,1) + (Xcorrect - Xdist);
        %korekcia presahu mimo sledovanu oblast %correction of overhang beyond tracked area
        if (sqrt((InTrajL(j,1) - CentPoint(1))^2 + (InTrajL(j,2) - CentPoint(2))^2) > HorDistLim)
            XYdist = sqrt((PrevPoint(1) - InTrajL(j,1))^2 + (PrevPoint(2) - InTrajL(j,2))^2);
            XYfromcent = sqrt((PrevPoint(1) - CentPoint(1))^2 + (PrevPoint(2) - CentPoint(2))^2);

            nfactor2 = XYdist/XYfromcent;
            Ydist2 = CentPoint(2) - PrevPoint(2);
            Xdist2 = CentPoint(1) - PrevPoint(1);
            Ycorrect2 = Ydist2 * nfactor2;
            Xcorrect2 = Xdist2 * nfactor2;

            InTrajL(j,2) = PrevPoint(2) + Ycorrect2;
            InTrajL(j,1) = PrevPoint(1) + Xcorrect2;
        end

        CircTrjDist = sqrt((CircPoints(:,1) - InTrajL(j,1)).^2 + (CircPoints(:,2) - InTrajL(j,2)).^2 + (CircPoints(:,3) - InTrajL(j,3)).^2);
        CircTrjMinDist = min(CircTrjDist(CircTrjDist<(RDist - TerrModRes),:));
        if (~isempty(CircTrjMinDist))
            MinDistCoor = CircPoints(CircTrjMinDist == CircTrjDist,:);
        end 
    end   
end
%konecny prevod finalnych hodnot z lokalnej do polarnej sustavy 
%final points conversion from local to polar frame
[OutTraj(:,1),OutTraj(:,2),OutTraj(:,3)] = enu2geodetic(InTrajL(:,1),InTrajL(:,2),InTrajL(:,3),InTraj(1,1),InTraj(1,2),InTraj(1,3),wgs84Ellipsoid);
%volitelne filtrovanie bodov %optional waypoint filtering
if (nargin == 4)
    WPSepar = zeros(size(InTrajL,1)-1,1);
    for i = 1:(size(InTrajL,1) - 1)
        WPSepar(i) = sqrt((InTrajL(i,1)-InTrajL(i+1,1))^2 + (InTrajL(i,2)-InTrajL(i+1,2))^2);
    end
    for i = 2:size(OutTraj,1)-1
        if (WPSepar(i) < MinWPSepar)
            OutTraj(i,:) = 0;
        end
    end
    OutTraj(~any(OutTraj,2),:) = [];
end
end