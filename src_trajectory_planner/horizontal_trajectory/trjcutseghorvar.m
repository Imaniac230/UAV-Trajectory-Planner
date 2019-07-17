function [CtrajP,NofWp,MapDist,WpGrad,TerrTraj] = trjcutseghorvar(InTraj,TerrMod,GradCmpTol,varargin)
%TRJCUTSEGHORVAR - Cut trajectory into variable smaller segments
%
%   This function cuts the input trajectory [InTraj] into segments of variable length
%   defined by the gradient in the vertical axis and returns the final waypoints in a matrix [CtrajP].
%   A digital elevation model [TerrMod] is required in order to  map the input trajectory to its
%   corresponding height. The number of calculated waypoints is returned in [NofWp].
%   The separation distances between trajectory points and treir corresponding mapped terrain height
%   model points in meters are returned in [MapDist]. A vector of calculated gradients of
%   corresponding waypoints in the longitudinal direction is returned in [WpGrad]. The cut out of the
%   terrain height model corresponding to the generated trajectory is returned in [TerrTraj].
%   The final number of generated waypoints is controlled by [GradCmpTol], which defines the tolerance of gradient difference.
%   The optional perpendicular distance [SafeDist] specifies a distance perpendicular to the trajectory in which
%   the waypoint placement should be sensitive to gradient differences [GradCmpTol].
%
%   [CtrajP,NofWp,MapDist,WpGrad,TerrTraj] = TRJCUTSEGHORVAR(InTraj,TerrMod,GradCmpTol)
%   [CtrajP,NofWp,MapDist,WpGrad,TerrTraj] = TRJCUTSEGHORVAR(InTraj,TerrMod,GradCmpTol,SafeDist)
%
%   The input trajectory [InTraj] must be a matrix of geodetic (polar) coordinates with 2 columns [latitude longitude].
%   The elevation model [TerrMod] is a raster given in geodetic (polar) coordinates [latitude longitude height].
%   The tolerance of gradient difference [GradCmpTol] and the optional perpendicular distance [SafeDist] parameter
%   must be positive values.

%%
%
InitSeg = 0.5; % [m]
%%
%inicializacne parametre %initial parameters
errTraj = 'Invalid input trajectory (1st parameter). Input must be a matrix of polar coordinates with 2 or 3 columns. For more info please visit help.';
errModel = 'Invalid terrain elevation model (2nd parameter). Input must be a matrix of polar coordinates with 3 columns. For more info please visit help.';
errGradtol = 'Invalid gradient difference tolerance (3rd parameter). Input must be a positive value. For more info please visit help.';
errDist = 'Invalid perpendicular distance (last parameter). Input must be a positive value. For more info please visit help.';
if ((size(TerrMod,2) ~= 3) || ~isnumeric(TerrMod))
    error(errModel)
end
if (~isnumeric(InTraj) || ((size(InTraj,2) ~= 2) && (size(InTraj,2) ~= 3)))
    error(errTraj)
end
if ((~isnumeric(GradCmpTol)) || (sum(size(GradCmpTol)) ~= 2) || (GradCmpTol < 0))
    error(errGradtol)
end
if (size(varargin,2) > 1)
    error('Too many input arguments.')
end
if (nargin == 4)
    if (~isnumeric(varargin{1}) || sum(size(varargin{1}) ~= 2) || varargin{1} < 0)
        error(errDist)
    end
    SafeDist = varargin{1};
end
%%
%namapovanie na vyskovy model %mapping to terrain height model
Traj2Grad = trjcutseghor(InTraj,InitSeg);
[Traj2Grad,TerrTrajFull,MapDistFull] = trjmap2dem(Traj2Grad,TerrMod);
[WPdist] = trjstats(Traj2Grad);
%prevod z polarnej na lokalnu sustavu %conversion from polar to local frame
[Traj2GradL(:,1),Traj2GradL(:,2)] = geodetic2enu(Traj2Grad(:,1),Traj2Grad(:,2),0,Traj2Grad(1,1),Traj2Grad(1,2),0,wgs84Ellipsoid);
Traj2GradL(:,3) = Traj2Grad(:,3);
%%
%orezanie trajektorie na segmenty %cuttning trajectory into segments
DistVector = zeros(1,size(Traj2Grad,1));
for i = 2:size(DistVector,2)
    DistVector(i) = sum(WPdist(1:i-1));
end
TrjGrad = gradient(Traj2GradL(:,3),DistVector);
CTrajL = zeros(size(Traj2GradL));
CTrajL(1,:) = Traj2GradL(1,:);
WpGrad = zeros(1,size(CTrajL,1));
WpGrad(1) = TrjGrad(1);
TerrTraj(1,:) = TerrTrajFull(1,:);
MapDist = zeros(1,size(CTrajL,1));
MapDist(1) = MapDistFull(1);
PrevIndex = 1;

switch nargin
    %jednorozmerna orientacia %one dimensional orientation
    case 3
        for i = 2:size(Traj2GradL,1)
            if ((abs(abs(TrjGrad(i)) - abs(TrjGrad(i-1))) > GradCmpTol) || isequal(ismembertol(Traj2Grad(i,1:2),InTraj,0.000000001),[1 1]))
                PrevIndex = PrevIndex + 1;
                CTrajL(PrevIndex,:) = Traj2GradL(i,:);
                WpGrad(PrevIndex) = TrjGrad(i);
                TerrTraj(PrevIndex,:) = TerrTrajFull(i,:);
                MapDist(PrevIndex) = MapDistFull(i);
            end
        end
        CTrajL = CTrajL(any(CTrajL ~= 0,2),:);
        MapDist(:,~any(MapDist,1)) = [];
        WpGrad(:,~any(WpGrad,1)) = [];
    %troj rozmerna orientacia %three dimensional orientation
    case 4
        %rotacia trajektorie %trajectory rotation
        angleR = atan(Traj2GradL(2,2)/Traj2GradL(2,1));
        R = [cos(-angleR) -sin(-angleR) 0; sin(-angleR) cos(-angleR) 0; 0 0 1];
        RefLtmp = Traj2GradL';
        Traj2GradL = (R*RefLtmp)';
        %korekcia orientacie %orientation correction
        if (Traj2GradL(3,2) < 0 && Traj2GradL(4,2) < 0)
            angleR = angleR + pi;
            R = [cos(-angleR) -sin(-angleR) 0; sin(-angleR) cos(-angleR) 0; 0 0 1];
            Traj2GradL = (R*RefLtmp)';
        end
        %vytvorenie kolmych vektorov %perpendicular vectors creation
        SafeCount = round(SafeDist/InitSeg);
        SafeV = zeros(2*SafeCount+1,3);
        SafeVfull = zeros(size(Traj2GradL,1)*size(SafeV,1),3);
        %urcenie poloh kolmych vektorov %perpendicular vectors positions location
        if (abs(Traj2GradL(1,2) - Traj2GradL(2,2)) < 0.000001)
            SafeV(:,1) = Traj2GradL(1,1);
            SafeV(1:SafeCount,2) = Traj2GradL(1,2)-SafeDist:InitSeg:Traj2GradL(1,2)-InitSeg;
            SafeV(SafeCount+2:end,2) = Traj2GradL(1,2)+InitSeg:InitSeg:Traj2GradL(1,2)+SafeDist;
        else
            SafeV(1:SafeCount,1) = Traj2GradL(1,1)-SafeDist:InitSeg:Traj2GradL(1,1)-InitSeg;
            SafeV(SafeCount+2:end,1) = Traj2GradL(1,1)+InitSeg:InitSeg:Traj2GradL(1,1)+SafeDist;
            SafeV(:,2) = Traj2GradL(1,2);
        end
        SafeV(SafeCount+1,:) = Traj2GradL(1,:);
        SafeVfull(1:size(SafeV,1),:) = SafeV;
        for i = 2:size(Traj2GradL,1)
            SafeV = zeros(2*SafeCount+1,3);
            if (abs(Traj2GradL(i-1,2) - Traj2GradL(i,2)) < 0.000001)
                SafeV(:,1) = Traj2GradL(i,1);
                SafeV(1:SafeCount,2) = Traj2GradL(i,2)-SafeDist:InitSeg:Traj2GradL(i,2)-InitSeg;
                SafeV(SafeCount+2:end,2) = Traj2GradL(i,2)+InitSeg:InitSeg:Traj2GradL(i,2)+SafeDist;
            else
                SafeV(1:SafeCount,1) = Traj2GradL(i,1)-SafeDist:InitSeg:Traj2GradL(i,1)-InitSeg;
                SafeV(SafeCount+2:end,1) = Traj2GradL(i,1)+InitSeg:InitSeg:Traj2GradL(i,1)+SafeDist;
                SafeV(:,2) = Traj2GradL(i,2);
            end
            SafeV(SafeCount+1,:) = Traj2GradL(i,:);
            SafeVfull((i-1)*size(SafeV,1)+1:i*size(SafeV,1),:) = SafeV;
        end
        %namapovanie vektorov na vyskovy model terenu %mapping vectors to terrain height model
        R = [cos(angleR) -sin(angleR) 0; sin(angleR) cos(angleR) 0; 0 0 1];
        vtmp = SafeVfull';
        SafeVfullR = (R*vtmp)';
        SafeVfullP = zeros(size(SafeVfullR,1),3);
        [SafeVfullP(:,1),SafeVfullP(:,2)] = enu2geodetic(SafeVfullR(:,1),SafeVfullR(:,2),0,Traj2Grad(1,1),Traj2Grad(1,2),0,wgs84Ellipsoid);
        SafeVfullP = trjmap2dem(SafeVfullP,TerrMod);
        %umiestnenie waypointov podla zmien gradientu v pozdlznom a kolmom smere
        %placing waypoints according to gradient changes in longitudinal and perpendicular directions
        SafeVglast = gradient(SafeVfullP(1:1*size(SafeV,1),3));
        for i = 2:size(Traj2GradL,1)
            SafeV = SafeVfullP((i-1)*size(SafeV,1)+1:i*size(SafeV,1),3);
            SafeVgrad = gradient(SafeV,InitSeg);
            %detekcia zmeny gradientov vektorov v pozdlznom smere
            %detection of gradient changes in vectors in the longitudinal direction
            SetPt = false;
            if (max(abs(abs(SafeVgrad) - abs(SafeVglast))) > GradCmpTol)
                SafeVgleft = SafeVgrad(1:SafeCount);
                SafeVgright = SafeVgrad(SafeCount+2:end);
                SafeVgleft = sign(sum(SafeVgleft(SafeVgleft>=0)) + sum(SafeVgleft(SafeVgleft<0)));
                SafeVgright = sign(sum(SafeVgright(SafeVgright>=0)) + sum(SafeVgright(SafeVgright<0)));
                %urcenie profilu terenu a detekcia zmeny gradientu v kolmom smere
                %determination of the terrain profile and detection of gradient changes in the perpendicular direction
                if (SafeCount == 1)
                    if (((SafeVgleft < 0) && (SafeVgright > 0)) && (max(abs(diff(abs(SafeVgrad)))) > GradCmpTol))
                        SetPt = true;
                    elseif (((SafeVgleft > 0) && (SafeVgright > 0)) && (max(abs(diff(abs(SafeVgrad(SafeCount+1:end))))) > GradCmpTol))
                        SetPt = true;
                    elseif ((SafeVgleft < 0) && (SafeVgright < 0) && (max(abs(diff(abs(SafeVgrad(1:SafeCount+1))))) > GradCmpTol))
                        SetPt = true;
                    end
                else
                    if (((SafeVgleft < 0) && (SafeVgright > 0)) && (max(abs(diff(abs(SafeVgrad)))) > GradCmpTol))
                        SetPt = true;
                    elseif (((SafeVgleft > 0) && (SafeVgright > 0)) && (max(abs(diff(abs(SafeVgrad(SafeCount+2:end))))) > GradCmpTol))
                        SetPt = true;
                    elseif ((SafeVgleft < 0) && (SafeVgright < 0) && (max(abs(diff(abs(SafeVgrad(1:SafeCount))))) > GradCmpTol))
                        SetPt = true;
                    end
                end
            end
            SafeVglast = SafeVgrad;
            %umiestnenie bodu %waypoint placement
            if ((abs(abs(TrjGrad(i)) - abs(TrjGrad(i-1))) > GradCmpTol) || isequal(ismembertol(Traj2Grad(i,1:2),InTraj,0.000000001),[1 1]) || (SetPt))
                PrevIndex = PrevIndex + 1;
                CTrajL(PrevIndex,:) = Traj2GradL(i,:);
                WpGrad(PrevIndex) = TrjGrad(i);
                TerrTraj(PrevIndex,:) = TerrTrajFull(i,:);
                MapDist(PrevIndex) = MapDistFull(i);
            end
        end
        CTrajL = CTrajL(any(CTrajL ~= 0,2),:);
        MapDist(:,~any(MapDist,1)) = [];
        WpGrad(:,~any(WpGrad,1)) = [];
        %opacna rotacia trajektorie %reverse trajectory rotation
        R = [cos(angleR) -sin(angleR) 0; sin(angleR) cos(angleR) 0; 0 0 1];
        FinLtmp = CTrajL';
        CTrajL = (R*FinLtmp)';
    otherwise
        error('Wrong number of input arguments.')
end
%finalny prevod z lokalnej na polarnu sustavu %final conversion from local to polar frame
[CtrajP(:,1),CtrajP(:,2)] = enu2geodetic(CTrajL(:,1),CTrajL(:,2),0,Traj2Grad(1,1),Traj2Grad(1,2),0,wgs84Ellipsoid);
CtrajP(:,3) = CTrajL(:,3);
%pocet finalnych bodov %number of final waypoints
NofWp = size(CtrajP,1);
end