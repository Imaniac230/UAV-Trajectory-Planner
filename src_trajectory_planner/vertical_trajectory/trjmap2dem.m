function [OutTraj,TerrTraj,MapDist] = trjmap2dem(InTraj,TerrMod,varargin)
%TRJMAP2DEM - Map trajectory to a digital elevation model
%
%   This function synchronizes the input trajectory points [InTraj] with corresponding
%   points in the digital elevation model [TerrMod]. 
%   Output is a cut out of the corresponding mapped elevation model points [TerrTraj], the trajectory
%   points with the mapped height levels [OutTraj] and the separation distance between trajectory points
%   and treir corresponding mapped terrain height model points [MapDist] in meters. 
%   Optional parameters [DistTol] is used to explicitly specify the maximum allowed tolerance.
%
%   [OutTraj,TerrTraj,MapDist] = TRJMAP2DEM(InTraj,TerrMod)
%   [OutTraj,TerrTraj,MapDist] = TRJMAP2DEM(InTraj,TerrMod,DistTol)
%
%   The input trajectory must be a matrix of geodetic (polar) coordinates with 2 columns [latitude longitude]
%   or 3 columns [latitude longitude height]. The elevation model is a raster given in geodetic (polar)
%   coordinates [latitude longitude height]. The maximum tolerance of distance between a trajectory way point
%   and a mapped terrain height model point [DistTol] must be a positive non-zero value and is given in meters [m].

%%
%inicializacne parametre %initial parameters
errTraj = 'Invalid input trajectory (1st parameter). Input must be a matrix of polar coordinates with 2 or 3 columns. For more info please visit help.';
errModel = 'Invalid elevation model (2nd parameter). Input must be a matrix of polar coordinates with 3 columns. For more info please visit help.';
errDist = 'Invalid tolerance (3rd parameter). Input must be a non-zero positive value. For more info please visit help.';
if (~isnumeric(InTraj) || (size(InTraj,2) ~= 2 && size(InTraj,2) ~= 3))
    error(errTraj)
end
if ((size(TerrMod,2) ~= 3) || ~isnumeric(TerrMod))
    error(errModel)
end
if (nargin > 3)
    error('Too many input parameters.')
end
if (nargin == 3)
    if (~isnumeric(varargin{1}) || (sum(size(varargin{1})) ~= 2) || (varargin{1} <= 0))
        error(errDist)
    end
    DistTol = varargin{1};
else
    DistTol = demresol(TerrMod,10);
end

flag = 0;
OutTraj = InTraj;
TerrTraj = zeros(size(InTraj,1),3);
MapDist = zeros(size(InTraj,1),1);
%%
%prevod z polarnej na lokalnu sustavu %conversion from polar to local frame
InTrajL = zeros(size(InTraj,1),3);
TerrModL = zeros(size(TerrMod,1),3);
[InTrajL(:,1),InTrajL(:,2)] = geodetic2enu(InTraj(:,1),InTraj(:,2),0,InTraj(1,1),InTraj(1,2),0,wgs84Ellipsoid);
[TerrModL(:,1),TerrModL(:,2)] = geodetic2enu(TerrMod(:,1),TerrMod(:,2),0,InTraj(1,1),InTraj(1,2),0,wgs84Ellipsoid);
%synchronizacia trajektorie s vyskovym modelom %trajectory and height model synchronization
%[k,sep] = dsearchn(TerrModL,InTrajL);
LastGoodi = 1;
for i = 1:size(InTraj,1)
    ModRange = sqrt((TerrModL(:,1) - InTrajL(i,1)).^2 + (TerrModL(:,2) - InTrajL(i,2)).^2);
    [ModMin,mindex] = min(ModRange);
    MapDist(i) = ModMin;
    if (ModMin < DistTol)
        OutTraj(i,3) = TerrMod(mindex,3);
        TerrTraj(i,:) = TerrMod(mindex,:);
        LastGoodi = i;
        flag = 1;
    else
        ModRange = sqrt((TerrModL(:,1) - InTrajL(LastGoodi,1)).^2 + (TerrModL(:,2) - InTrajL(LastGoodi,2)).^2);
        [~,mindex] = min(ModRange);
        TerrTraj(i,:) = TerrMod(mindex,:);
    end
end
%detekcia presahu tolerancie %out of tolerance detection
if (flag == 0)
    warning('MapToDEM:OutOfBounds','No waypoints were mapped to the digital elevation model according to the tolerance of %.6gm! Trajectory might be outside the raster!',DistTol)
end
if (MapDist(:,1) == 0)
    MapDist(:,1) = NaN;
end
badpoints = MapDist((MapDist > DistTol) | (MapDist == 0));
if (~isempty(badpoints))
    warning('MapToDEM:OutOfBounds','%d waypoints were not mapped to the digital elevation model according to the tolerance of %.6gm! The trajectory might be out of bounds of the terrain height model!',size(badpoints,1),DistTol)
end
end