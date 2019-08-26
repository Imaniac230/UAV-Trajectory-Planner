function [RealTrjOut,RealTrjDistOut,TerrTrjOut,TerrTrjDistOut,iAll] = analyzeFlight_DJI(RealTrj,PlanTrj,TerrMod,varargin)
%ANALYZEFLIGHT_DJI - Analyze flight data from the DJI drone
%
%   This function compares the real flight trajecotry of a DJI drone [Realatjr] with the planned trajectory [PlanTrj] and a given digital
%   elevation model [TerrMod] and plots them out. Plots can be specified by optional parameters.
%   Optional parameters include: 'TrajFindPrec', 'TerrainRes', 'RealTrajMode', 'TerrainMode', 'PlanTrajMode', 'DemRes', 'ShowPlots'
%   Additionaly indexes of the located mission trajectory points are returned in [iAll] so that:
%       RealTrj(iAll,3) == RealTrjOut or RealTrj(iAll(1):iAll(end),3) == RealTrjOut.
%
%   [RealTrjOut,RealTrjDistOut,TerrTrjOut,TerrTrjDistOut,iAll] = ANALYZEFLIGHT_DJI(RealTrj,PlanTrj,TerrMod)
%   [RealTrjOut,RealTrjDistOut,TerrTrjOut,TerrTrjDistOut,iAll] = ANALYZEFLIGHT_DJI(RealTrj,PlanTrj,TerrMod,ParName1,ParValue1,...,ParNamen,ParValuen)
%
%   Input trajectories [RealTrj], [PlanTrj] and terrain model [TerrMod] must be matrices of geodetic (polar) coordinates of format
%   [latitude longitude height], where height must be a positive value in [m]. Terrain model resolution [TerrModRes] must be a
%   positive value given in meters [m]. Optional parameters have the following requirements:
%
%   'TrajFindPrec' defines the tolerance with which the first and last points of the trajectory are found and must be a single positive value
%                   given in meters [m]
%                   (default: 0.5m)
%
%   'TerrainRes' defines the resolution of the plotted terrain profile and must be a single positive value given in meters [m]
%                   (default: 1m)
%
%   'RealTrajMode' must be a single integer value from 0 to 3, where:
%                   [0->not plotted, 1->plotted from full interpolated trajectory, 2->plotted from full unmodified trajectory
%                   3->plotted only from points closest to planned trajectory]
%                   (default: 2)
%
%   'TerrainMode' must be a single integer value from 0 to 2, where:
%                   [0->not plotted, 1->terrain under real trajectory, 2->terrain under planned trajectory]
%                   (default: 1)
%
%   'PlanTrajMode' must be a single integer value from 0 to 1, where:
%                   [0->not plotted, 1->plotted]
%                   (default: 1)
%
%   'DemRes' defines the resolution of the used elevation model given in meters [m]
%                   (default: implicit)
%
%   'ShowPlots' must be a single integer value from 0 to 1, where:
%                   [0->plots not displayed, 1->plots displayed]
%                   (default: 1)

%%
%inicializacne parametre %initial parameters
AllowedParams = {'TrajFindPrec', 'TerrainRes', 'RealTrajMode', 'TerrainMode', 'PlanTrajMode', 'DemRes', 'ShowPlots'};

errargs = sprintf('''%s'', ', AllowedParams{1:end-1});
errargs = sprintf('%s''%s''. ',errargs, AllowedParams{end});
errArgs = 'Incorrect optional parameters. \nAllowed strings are: %s\nFor more info please visit help.';
errReal = 'Invalid input trajectory (1st parameter). Input must be a three column matrix containing geodetic coordinates. For more info please visit help.';
errPlan = 'Invalid input trajectory (2nd parameter). Input must be a three column matrix containing geodetic coordinates. For more info please visit help.';
errHeightR = 'Invalid height in real trajectory (1st parameter). One or more waypoints have negative height values. For more info please visit help.';
errHeightP = 'Invalid height in planned trajectory (2nd parameter). One or more waypoints have negative height values. For more info please visit help.';
errCreate = 'Not enough input arguments. Please specify names and corresponding values for given optional parameters if used. For more info please visit help.';
errParSize = '%dth parameter exceeds maximum dimensions. Parameter must be a single value. For more info please visit help.';
errParType = 'Invalid %dth parameter. Parameter must be a numeric value. For more info please visit help.';
errFindPrec = 'Invalid trajectory locating precision (%dth parameter). Parameter must be a positive value. For more info please visit help.';
errTerrRes = 'Invalid terrain plot resolution (%dth parameter). Parameter must be a positive value. For more info please visit help.';
errRealMode = 'Invalid real trajectory plot mode (%dth parameter). Parameter must be an integer between 0 an 3. For more info please visit help.';
errTerrMode = 'Invalid real terrain plot mode (%dth parameter). Parameter must be an integer between 0 an 2. For more info please visit help.';
errPlanMode = 'Invalid planned trajectory plot mode (%dth parameter). Parameter must be an integer between 0 an 1. For more info please visit help.';
errDemRes = 'Invalid elevation model resolution (%dth parameter). Parameter must be a positive value in [m]. For more info please visit help.';
errPlots = 'Invalid show plots option (%dth parameter). Parameter must be an integer between 0 an 1. For more info please visit help.';
errFind = 'Could not synchronize real and planned trajectories within the specified tolerance %.6gm. Please make sure the inputs are correct or increase the tolerance.';

PlanTrjLegend = 'Planned trajectory waypoints';
RealTrjLegend = 'Real trajectory from DJI Phantom 3 Adv';
TerrLegend = 'Terrain gl';
%%
%defaultne hodnoty volitelnych parametrov %default values of optional parameters
TrjFindPrec = 0.5;
TerrPlotRes = 1;
RealTrjPlot = 2;
TerrPlot = 1;
PlanTrjPlot = 1;
ShowPlots = 1;
%overenie parametrov %input parameters verification
if ((size(RealTrj,2) ~= 3) || ~isnumeric(RealTrj))
    error(errReal)
end
if ((size(PlanTrj,2) ~= 3) || ~isnumeric(PlanTrj))
    error(errPlan)
end
if (mod(size(varargin,2),2) ~= 0)
    error(errCreate)
end
if (min(ismember(varargin(1:2:end),AllowedParams)) < 1)
    error(errArgs,errargs)
end
if (max(RealTrj(:,3) < 0) > 0)
    error(errHeightR)
end
if (max(PlanTrj(:,3) < 0) > 0)
    error(errHeightP)
end

for i = 2:2:size(varargin,2)
    if(~isnumeric(varargin{i}))
        error(errParType,i+3)
    end
    if ((size(varargin{i},1) > 1 && size(varargin{i},2) > 1))
        error(errParSize,i+3)
    end
    
    switch varargin{i-1}
        case 'TrajFindPrec'
            if (varargin{i} <= 0)
                error(errFindPrec,i+3)
            end
            TrjFindPrec = varargin{i};
        case 'TerrainRes'
            if (varargin{i} <= 0)
                error(errTerrRes,i+3)
            end
            TerrPlotRes = varargin{i};
        case 'RealTrajMode'
            if ((mod(varargin{i},1) ~= 0) || (varargin{i} < 0) || (varargin{i} > 3))
                error(errRealMode,i+3)
            end
            RealTrjPlot = varargin{i};
        case 'TerrainMode'
            if ((mod(varargin{i},1) ~= 0) || (varargin{i} < 0) || (varargin{i} > 2))
                error(errTerrMode,i+3)
            end
            TerrPlot = varargin{i};
        case 'PlanTrajMode'
            if ((mod(varargin{i},1) ~= 0) || (varargin{i} < 0) || (varargin{i} > 1))
                error(errPlanMode,i+3)
            end
            PlanTrjPlot = varargin{i};
        case 'DemRes'
            if (varargin{i} <= 0)
                error(errDemRes,i+3)
            end
            DemRes = varargin{i};
        case 'ShowPlots'
            if ((mod(varargin{i},1) ~= 0) || (varargin{i} < 0) || (varargin{i} > 1))
                error(errPlots,i+3)
            end
            ShowPlots = varargin{i};
        otherwise
            error(errArgs,errargs)
    end
end
%%
%prevod z polarnej na lokalnu sustavu %conversion from polar to local frame
RealTrjL = zeros(size(RealTrj));
PlanTrjL = zeros(size(PlanTrj));
[RealTrjL(:,1),RealTrjL(:,2)] = geodetic2enu(RealTrj(:,1),RealTrj(:,2),0,PlanTrj(1,1),PlanTrj(1,2),0,wgs84Ellipsoid);
RealTrjL(:,3) = RealTrj(:,3);
[PlanTrjL(:,1),PlanTrjL(:,2)] = geodetic2enu(PlanTrj(:,1),PlanTrj(:,2),0,PlanTrj(1,1),PlanTrj(1,2),0,wgs84Ellipsoid);
PlanTrjL(:,3) = PlanTrj(:,3);
%%
%najdenie trajektorie %locating of the trajectory
iAll = zeros(size(PlanTrjL,1),1);
RealTrjL_backup = RealTrjL;
%pociatocny a konecny bod %first and final waypoint
RealTrjL((sqrt((RealTrjL(:,1) - PlanTrjL(1,1)).^2 + (RealTrjL(:,2) - PlanTrjL(1,2)).^2) < TrjFindPrec),4) = 1;
if (sum(RealTrjL(:,4)) == 0)
    error(errFind,TrjFindPrec)
end
iAll(1) = find(RealTrjL(:,4),1,'last');
RealTrjL(:,4) = 0;

RealTrjL((sqrt((RealTrjL(:,1) - PlanTrjL(end,1)).^2 + (RealTrjL(:,2) - PlanTrjL(end,2)).^2) < TrjFindPrec),4) = 1;
if (sum(RealTrjL(:,4)) == 0)
    error(errFind,TrjFindPrec)
end
iAll(end) = find(RealTrjL(:,4),1,'first');
RealTrjL(:,4) = 0;

if (iAll(1) > iAll(end))
    SecondCase = false;
    RealTrjL(1:iAll(1)-1,:) = NaN;
    
    RealTrjL((sqrt((RealTrjL(:,1) - PlanTrjL(1,1)).^2 + (RealTrjL(:,2) - PlanTrjL(1,2)).^2) < TrjFindPrec),4) = 1;
    if (sum(RealTrjL(:,4)) == 0)
        error(errFind,TrjFindPrec)
    end
    iAll(1) = find(RealTrjL(:,4),1,'last');
    RealTrjL(:,4) = 0;

    RealTrjL((sqrt((RealTrjL(:,1) - PlanTrjL(end,1)).^2 + (RealTrjL(:,2) - PlanTrjL(end,2)).^2) < TrjFindPrec),4) = 1;
    if (sum(RealTrjL(:,4)) == 0)
        SecondCase = true;
    end
    iAll(end) = find(RealTrjL(:,4),1,'first');
    RealTrjL(:,4) = 0;
    
    if (SecondCase)
        RealTrjL = RealTrjL_backup;
        RealTrjL(iAll(end)+1:end,:) = NaN;
        
        RealTrjL((sqrt((RealTrjL(:,1) - PlanTrjL(1,1)).^2 + (RealTrjL(:,2) - PlanTrjL(1,2)).^2) < TrjFindPrec),4) = 1;
        if (sum(RealTrjL(:,4)) == 0)
            error(errFind,TrjFindPrec)
        end
        iAll(1) = find(RealTrjL(:,4),1,'last');
        RealTrjL(:,4) = 0;

        RealTrjL((sqrt((RealTrjL(:,1) - PlanTrjL(end,1)).^2 + (RealTrjL(:,2) - PlanTrjL(end,2)).^2) < TrjFindPrec),4) = 1;
        if (sum(RealTrjL(:,4)) == 0)
            error(errFind,TrjFindPrec)
        end
        iAll(end) = find(RealTrjL(:,4),1,'first');
        RealTrjL(:,4) = 0;
    end
end
%zvysne body %all other waypoints
RealTrjL(:,4) = 0;
for i=2:(size(PlanTrjL,1)-1)
    iFound = 1:size(RealTrj,1);
    iFound = iFound(((iFound > iAll(1)) + (iFound < iAll(end))) == 2);
    [~,iFoundmin] = min(sqrt((RealTrjL(iFound,1) - PlanTrjL(i,1)).^2 + (RealTrjL(iFound,2) - PlanTrjL(i,2)).^2));
    iAll(i) = iFound(iFoundmin);
    RealTrjL(:,4) = 0;
end
RealTrjL(:,4) = [];
%%
%najdene najblizsie body k planovanej trajektorii %closest found points to planned trajerctory
RealTrjP_shrunk(:,1) = RealTrj(iAll,1);
RealTrjP_shrunk(:,2) = RealTrj(iAll,2);
RealTrjP_shrunk(:,3) = RealTrj(iAll,3);

RealTrjL_shrunk(:,1) = RealTrjL(iAll,1);
RealTrjL_shrunk(:,2) = RealTrjL(iAll,2);
RealTrjL_shrunk(:,3) = RealTrjL(iAll,3);
%horizontalne posunutie bodov pre profilovy graf %horizontal displacement of waypoints for profile plot
if (RealTrjPlot == 1)
    RealTrjL_modified = RealTrjL;
    for i=1:size(RealTrjL_shrunk,1)-1
        c = sqrt((RealTrjL(iAll(i),1) - RealTrjL(iAll(i+1),1))^2 + (RealTrjL(iAll(i),2) - RealTrjL(iAll(i+1),2))^2);%su spravne
        for j=(iAll(i)+1):(iAll(i+1)-1)
            a = sqrt((RealTrjL(iAll(i),1) - RealTrjL(j,1))^2 + (RealTrjL(iAll(i),2) - RealTrjL(j,2))^2);
            b = sqrt((RealTrjL(iAll(i+1),1) - RealTrjL(j,1))^2 + (RealTrjL(iAll(i+1),2) - RealTrjL(j,2))^2);

            s = (a+b+c)/2;
            S = sqrt(s*(s-a)*(s-b)*(s-c));
            Vc = 2*S/c;
            n = Vc/5;
            coor1 = n*3;
            coor2 = n*4;

            test_point(1,1) = RealTrjL(j,1) + coor2;
            test_point(1,2) = RealTrjL(j,2) - coor1;
            test_point(2,1) = RealTrjL(j,1) - coor2;
            test_point(2,2) = RealTrjL(j,2) + coor1;
            test_point(3,1) = RealTrjL(j,1) - coor2;
            test_point(3,2) = RealTrjL(j,2) - coor1;
            test_point(4,1) = RealTrjL(j,1) + coor2;
            test_point(4,2) = RealTrjL(j,2) + coor1;

            test_point(5,1) = RealTrjL(j,1) + coor1;
            test_point(5,2) = RealTrjL(j,2) - coor2;
            test_point(6,1) = RealTrjL(j,1) - coor1;
            test_point(6,2) = RealTrjL(j,2) + coor2;
            test_point(7,1) = RealTrjL(j,1) - coor1;
            test_point(7,2) = RealTrjL(j,2) - coor2;
            test_point(8,1) = RealTrjL(j,1) + coor1;
            test_point(8,2) = RealTrjL(j,2) + coor2;

            test_side1 = sqrt((test_point(:,1)-RealTrjL(iAll(i),1)).^2 + (test_point(:,2)-RealTrjL(iAll(i),2)).^2);
            test_side2 = sqrt((test_point(:,1)-RealTrjL(iAll(i+1),1)).^2 + (test_point(:,2)-RealTrjL(iAll(i+1),2)).^2);
            testc = test_side1 + test_side2;
            [~,idx] = min(testc - c);

            RealTrjL_modified(j,1:2) = test_point(idx,1:2);
        end
    end
    RealTrjL_modified = RealTrjL_modified(iAll(1):iAll(end),:);
end
%%
%parametre pre grafy %parameters for graphs
if (RealTrjPlot == 3)
    RealTrj_shrunk_dist = zeros(size(RealTrjL_shrunk,1),1);
    for i=2:size(RealTrj_shrunk_dist,1)
        RealTrj_shrunk_dist(i) = RealTrj_shrunk_dist(i-1) + sqrt((RealTrjL_shrunk(i-1,1)-RealTrjL_shrunk(i,1))^2 + (RealTrjL_shrunk(i-1,2)-RealTrjL_shrunk(i,2))^2);
    end
elseif (RealTrjPlot == 1)
    RealTrj_modified_dist = zeros(size(RealTrjL_modified,1),1);
    for i=2:size(RealTrj_modified_dist,1)
        RealTrj_modified_dist(i) = RealTrj_modified_dist(i-1) + sqrt((RealTrjL_modified(i-1,1)-RealTrjL_modified(i,1))^2 + (RealTrjL_modified(i-1,2)-RealTrjL_modified(i,2))^2);
    end
elseif (RealTrjPlot == 2)
    RealTrjL_unmodified = RealTrjL(iAll(1):iAll(end),:);
    RealTrj_unmodified_dist = zeros(size(RealTrjL_unmodified,1),1);
    for i=2:size(RealTrj_unmodified_dist,1)
        RealTrj_unmodified_dist(i) = RealTrj_unmodified_dist(i-1) + sqrt((RealTrjL_unmodified(i-1,1)-RealTrjL_unmodified(i,1))^2 + (RealTrjL_unmodified(i-1,2)-RealTrjL_unmodified(i,2))^2);
    end
end

if (PlanTrjPlot > 0)
    PlanTrj_dist = zeros(size(PlanTrjL,1),1);
    for i=2:size(PlanTrj_dist,1)
        PlanTrj_dist(i) = PlanTrj_dist(i-1) + sqrt((PlanTrjL(i-1,1)-PlanTrjL(i,1))^2 + (PlanTrjL(i-1,2)-PlanTrjL(i,2))^2);
    end
end

if (RealTrjPlot == 3)
    RealTrjOut = RealTrjP_shrunk;
    RealTrjDistOut = RealTrj_shrunk_dist;
elseif (RealTrjPlot == 1)
    RealTrjOut = RealTrj(iAll(1):iAll(end),:);
    RealTrjDistOut = RealTrj_modified_dist;
elseif (RealTrjPlot == 2)
    RealTrjOut = RealTrj(iAll(1):iAll(end),:);
    RealTrjDistOut = RealTrj_unmodified_dist;
else
    RealTrjDistOut = 0;
end

if (TerrPlot == 2)
    %pod planovanou trajektoriou %below planned trajectory
    [TerrTrjOut] = trjcutseghor(PlanTrj,TerrPlotRes);
    if (exist('DemRes','var'))
        [TerrTrjOut] = trjmap2dem(TerrTrjOut,TerrMod,DemRes);
    else
        [TerrTrjOut] = trjmap2dem(TerrTrjOut,TerrMod);
    end
    [terrdist] = trjstats(TerrTrjOut);
    TerrTrjDistOut = zeros(1,size(TerrTrjOut,1));
    for i=2:size(TerrTrjDistOut,2)
        TerrTrjDistOut(i) = sum(terrdist(1:i-1));
    end
elseif ((TerrPlot == 1) && (RealTrjPlot > 0))
    %pod skutocnou trajektoriou %below real trajectory
    [TerrTrjOut] = trjcutseghor(RealTrjOut,TerrPlotRes);
    if (exist('DemRes','var'))
        [TerrTrjOut] = trjmap2dem(TerrTrjOut,TerrMod,DemRes);
    else
        [TerrTrjOut] = trjmap2dem(TerrTrjOut,TerrMod);
    end
    if (RealTrjPlot == 1)
        TerrTrjDistOut = RealTrjDistOut;
    else
        [terrdist] = trjstats(TerrTrjOut);
        TerrTrjDistOut = zeros(1,size(TerrTrjOut,1));
        for i=2:size(TerrTrjDistOut,2)
            TerrTrjDistOut(i) = sum(terrdist(1:i-1));
        end
    end
else
    TerrTrjDistOut = 0;
    TerrTrjOut = [0 0 0];
end
%%
%vykreslenie grafov %plotting graphs
if (ShowPlots == 1)
    figure('Name','UAV Trajectory Planner (Flight Analysis)')
    subplot(2,2,1)
    plot(PlanTrjL(:,1),PlanTrjL(:,2),'b+')
    hold on
    plot(RealTrjL(iAll(1):iAll(end),1),RealTrjL(iAll(1):iAll(end),2),'g-')
    plot(PlanTrjL(1,1),PlanTrjL(1,2),'db','MarkerSize',10,'LineWidth',1.5)
    legend('Planned trajectory waypoints','Real trajectory from DJI Phantom 3 Adv','Starting waypoint','Location','best')
    xlabel('X-easting [m]');
    ylabel('Y-northing [m]');
    title('Planned vs. Real Trajectory (horizontal 2D view)')
    grid on
    % axis equal

    subplot(2,1,2)
    hold on
    if (PlanTrjPlot > 0)
        plot(PlanTrj_dist,PlanTrj(:,3),'b+:')
    end
    if (RealTrjPlot > 0)
        plot(RealTrjDistOut,RealTrjOut(:,3),'g-')
    end
    if (TerrPlot > 0)
        plot(TerrTrjDistOut,TerrTrjOut(:,3),'-r')
    end

    if (PlanTrjPlot > 0 && RealTrjPlot > 0)
        if (TerrPlot > 0)
            legend(PlanTrjLegend,RealTrjLegend,TerrLegend,'location','best')
        else
            legend(PlanTrjLegend,RealTrjLegend,'location','best')
        end
    elseif (PlanTrjPlot == 0 && RealTrjPlot > 0)
        if (TerrPlot > 0)
            legend(RealTrjLegend,TerrLegend,'location','best')
        else
            legend(RealTrjLegend,'location','best')
        end
    elseif (RealTrjPlot == 0 && PlanTrjPlot > 0)
        if (TerrPlot > 0)
            legend(PlanTrjLegend,TerrLegend,'location','best')
        else
            legend(PlanTrjLegend,'location','best')
        end
    end
    xlabel('Flight distance [m]');
    ylabel('Height MSL [m]');
    title('Planned vs. Real Trajectory (vertical 2D profile view)')
    grid on
    grid minor
end
end