function [OutTraj] = trjascentlim(InTraj,TrajSpeed,MaxRise)
%TRJASCENTLIM Verify trajectory with UAV's max vertical ascend capabilities
%   
%   This function calculates ascent angles between trajectory waypoints [InTraj] and compares them with the maximal ascent capabilities
%   of the UAV given by its maximal vertical velocity [MaxRise] and desired horizontal velocity for each waypoint [TrajSpeed].
%   If a calculated ascent angle exceeds the maximum ascent angle the function corrects the corresponding waypoint height.
%
%   [OutTraj] = TRJASCENTLIM(InTraj,TrajSpeed,MaxRise)
%
%   Input trajectory [InMat] must be a matrix of geodetic (polar) coordinates with 3 columns representing [latitude longitude height]
%   respectively. [TrajSpeed] must be a vector of positive values for each waypoint or a single positive value given in [m/s].
%   Maximum vertical ascent of the UAV [MaxRise] must be a positive value given in [m/s].

%%
%inicializacne parametre %initial parameters
errMat = 'Invalid input trajectory (1st parameter). Input must be a matrix of polar coordinates with 3 columns. For more info please visit help.';
errSpeed = 'Invalid vertical speed (2nd parameter). Input must be a non-zero positive number or an array of such numbers for each waypoint. For more info please visit help.';
errRise = 'Invalid max vertical speed (last parameter). Input must be a positive number. For more info please visit help.';
if ((size(InTraj,2) ~= 3) || ~isnumeric(InTraj))
    error(errMat)
end
if ((((sum(size(TrajSpeed))-1) ~= size(InTraj,1)) && (sum(size(TrajSpeed)) ~= 2)) || ~isnumeric(TrajSpeed) || (max(TrajSpeed <= 0) > 0))
    error(errSpeed)
end
if ((sum(size(MaxRise)) ~= 2) || ~isnumeric(MaxRise) || (MaxRise <= 0))
    error(errRise)
end
if (sum(size(TrajSpeed)) == 2)
    TrajSpeedtmp = TrajSpeed;
    TrajSpeed = zeros(size(InTraj,1));
    TrajSpeed(:) = TrajSpeedtmp;
end
%%
%prevod z polarnej na lokalnu sustavu %conversion from polar to local frame
InTrajL = zeros(size(InTraj));
[InTrajL(:,1),InTrajL(:,2),InTrajL(:,3)] = geodetic2enu(InTraj(:,1),InTraj(:,2),InTraj(:,3),InTraj(1,1),InTraj(1,2),InTraj(1,3),wgs84Ellipsoid);
%%
%overenie maximalneho stupania %maximal ascend verification
RiseAngmax = atan(MaxRise./TrajSpeed);
count = 0;
TrjOk = false;
while (~TrjOk)
    TrjOk = true;
    for i = 1:(size(InTrajL,1)-1)
        dist = sqrt((InTrajL(i+1,1) - InTrajL(i,1))^2 + ((InTrajL(i+1,2) - InTrajL(i,2)))^2 + ((InTrajL(i+1,3) - InTrajL(i,3)))^2);
        RiseAngact = asin(abs(InTrajL(i+1,3) - InTrajL(i,3))/dist);

        if ((RiseAngact - RiseAngmax(i)) > 0.001)
            TrjOk = false;
            count = count + 1;
            if (InTrajL(i+1,3) > InTrajL(i,3))
                InTrajL(i,3) = InTrajL(i+1,3) - dist*sin(RiseAngmax(i));
            else
                InTrajL(i+1,3) = InTrajL(i,3) - dist*sin(RiseAngmax(i));
            end
        end
    end
end
if (count > 0)
    warning('UAVAscentLimit:LimitExceeded','UAV''s ascend limit exceeded! %d corrections made to trajectory height.',count);
end
%%
%prevod finalnych hodnot z lokalnej do polarnej sustavy
%conversion from local to polar frame
[OutTraj(:,1),OutTraj(:,2),OutTraj(:,3)] = enu2geodetic(InTrajL(:,1),InTrajL(:,2),InTrajL(:,3),InTraj(1,1),InTraj(1,2),InTraj(1,3),wgs84Ellipsoid);
end