function [OutCell] = trj2csv_ugcs(InMat,HType,FileName,varargin)
%TRJ2CSV_UGCS - Create a custom trajectory csv file
%
%   This function creates a custom csv file named [FileName] specified by the trajectory waypoint data [InMat] altitude type [HType]
%   and other optional parameters.
%   Optional parameters include: 'Speed', 'Picture', 'ElevationMap', 'WP', 'CameraTilt', 'UAVYaw', 'DistanceFromFace', 'WaitTime'
%
%   [OutCell] = TRJ2CSV_UGCS(InMat,HType,FileName)
%   [OutCell] = TRJ2CSV_UGCS(InMat,HType,FileName,ParName1,ParValue1,...,ParNamen,ParValuen)
%
%   Input data [InMat] must be a matrix of geodetic (polar) coordinates of format [latitude longitude height], where height
%   must be a positive value in [m]. [HType] must be a string containing 'AltitudeAGL' or 'AltitudeAMSL' defining the type of height.
%   Optional parameters have the following requirements:
%
%   'Speed' must be a positive non-zero value or an array (vector,cell) of such values for specific waypoints in [m/s].
%   'Picture' must be a string containing 'TRUE' or 'FALSE' (case insensitive) or an array (cell) of such strings for specific waypoints.
%   'ElevationMap' ?
%   'WP' must be an array (vector,cell) of non-zero positive integers for each waypoint. Number of values in this parameter must
%   be the same as the number of waypoints in [InMat]!
%   'CameraTilt' must be a numeric value or an array (vector,cell) of such values for specific waypoints in [°].
%   'UAVYaw' must be a numeric value or an array (vector,cell) of such values for specific waypoints in [°].
%   'DistanceFromFace' ?
%   'WaitTime' must be a positive non-zero value or an array (vector,cell) of such values for specific waypoints in [s]. 
%
%   Maximum number of values for any optional parameter is limited to the number of waypoints in [InMat].
%   Arrays of optional parameters must be one-dimensional given as a single row or column.

%%
%inicializacne parametre %initial parameters
AllowedParams = {'Latitude' 'Longitude' 'AltitudeAMSL' 'AltitudeAGL' 'Speed' 'Picture' 'ElevationMap' ...
    'WP' 'CameraTilt' 'UavYaw' 'DistanceFromFace' 'WaitTime'};

errargs = sprintf('''%s'', ', AllowedParams{1:end-1});
errargs = sprintf('%s''%s''. ',errargs, AllowedParams{end});
errArgs = 'Incorrect optional parameters. \nAllowed strings are: %s\nFor more info please visit help.';
errData = 'Invalid input matrix (1st parameter). Input must be a three column matrix containing geodetic coordinates. For more info please visit help.';
errHeight = 'Invalid height in input matrix (1st parameter). One or more waypoints have negative height values. For more info please visit help.';
errCreate = 'Not enough input arguments. Please specify names and corresponding values for given optional parameters if used. For more info please visit help.';
errHType = 'Incorrect height mode parameter. Input must be either ''AltitudeAMSL'' or ''AltitudeAGL''. For more info please visit help.';
errParSize = '%dth parameter exceeds maximum dimensions. Parameter must be a one dimensional cell array or vector. For more info please visit help.';
errParLength = '%dth parameter exceeds maximum dimensions. Length of the aray must not be larger the number of given waypoints. For more info please visit help.';
errWPNLength = 'Waypoint number (%dth parameter) has wrong dimensions. Input must be an array of the same length as the number of given waypoints. For more info please visit help.';
errWPN = 'Invalid waypoint number (%dth parameter). Input must be an array of integers for each waypoint. For more info please visit help.';
errSpeed = 'Invalid UAV speed (%dth parameter). Input must be a non-zero positive number or an array of such numbers for multiple waypoints. For more info please visit help.';
errPic = 'Invalid picture mode (%dth parameter). Input must be a ''TRUE'' or ''FALSE'' string or a cell of such strings for multiple waypoints. For more info please visit help.';
errYaw = 'Invalid UAV yaw (%dth parameter). Input must be a number or an array of numbers for multiple waypoints. For more info please visit help.';
errTilt = 'Invalid camera tilt (%dth parameter). Input must be a number or an array of numbers for multiple waypoints. For more info please visit help.';
errTime = 'Invalid wait time (%dth parameter). Input must be a positive number or an array of such numbers for multiple waypoints. For more info please visit help.';

%errMap = 'Invalid elevation map (%dth parameter). Input must be a ? or an array of ? for multiple waypoints. For more info please visit help.';
%errFace = 'Invalid distance from face (%dth parameter). Input must be a ? or an array of ? for multiple waypoints. For more info please visit help.';
%%
%overenie parametrov %input parameters verification
if ((size(InMat,2) ~= 3) || ~isnumeric(InMat))
    error(errData)
end
if (~strcmp(HType,'AltitudeAMSL') && ~strcmp(HType,'AltitudeAGL'))
    error(errHType)
end
if (mod(size(varargin,2),2) ~= 0)
    error(errCreate)
end
if (min(ismember(varargin(1:2:end),AllowedParams)) < 1)
    error(errArgs,errargs)
end
if (max(InMat(:,3) < 0) > 0)
    error(errHeight)
end
ArgType = zeros(size(InMat,2)+size(varargin,2)/2,1);

for i = 2:2:size(varargin,2)
    if (~isa(varargin{i},'cell') && ~isa(varargin{i},'char'))
        varargin{i} = num2cell(varargin{i});
    elseif (isa(varargin{i},'char'))
        varargin(i) = num2cell(varargin(i));
    end
    if ((size(varargin{i},1) > 1 && size(varargin{i},2) > 1))
        error(errParSize,i+3)
    end
    if ((size(varargin{i},1) > size(InMat,1) || size(varargin{i},2) > size(InMat,1)))
        error(errParLength,i+3)
    end
    
    switch varargin{i-1}
        case 'Picture'
            ArgType(3+i/2) = 1; %string
            if (size(varargin{i},1) == 1 && size(varargin{i},2) == 1)
                if (~strcmpi(varargin{i},'TRUE') && ~strcmpi(varargin{i},'FALSE') && ~strcmpi(varargin{i},''))
                    error(errPic,i+3)
                end
            else
                try
                    if ((sum(ismember(upper(varargin{i}),'TRUE')) + sum(ismember(upper(varargin{i}),'FALSE')) + sum(ismember(upper(varargin{i}),''))) ~= size(varargin{i},2))
                        error(errPic,i+3)
                    end
                catch
                    error(errPic,i+3)
                end
            end
        case 'ElevationMap'
            ArgType(3+i/2) = 1; %string
        case 'DistanceFromFace'
            ArgType(3+i/2) = 1; %string
        case 'WP'
            if ((size(varargin{i},1) ~= size(InMat,1) && size(varargin{i},2) ~= size(InMat,1)))
                error(errWPNLength,i+3)
            end
            ArgType(3+i/2) = 2; %integer
            try
                if ((max(mod(cell2mat(varargin{i}),1)) ~= 0) || (~isnumeric(cell2mat(varargin{i}))))
                    error(errWPN,i+3)
                end
            catch
                error(errWPN,i+3)
            end
        case 'Speed'
            try
                if ((max(cell2mat(varargin{i}) <= 0) > 0) || (~isnumeric(cell2mat(varargin{i}))))
                    error(errSpeed,i+3)
                end
            catch
                error(errSpeed,i+3)
            end
        case 'UavYaw'
            try
                if (~isnumeric(cell2mat(varargin{i})))
                    error(errYaw,i+3)
                end
            catch
                error(errYaw,i+3)
            end
        case 'CameraTilt'
            try
                if (~isnumeric(cell2mat(varargin{i})))
                    error(errTilt,i+3)
                end
            catch
                error(errTilt,i+3)
            end
        case 'WaitTime'
            try
                if ((max(cell2mat(varargin{i}) <= 0) > 0) || (~isnumeric(cell2mat(varargin{i}))))
                    error(errTime,i+3)
                end
            catch
                error(errTime,i+3)
            end
    end
end
%%
%vytvorenie csv suboru %csv file creation
InMatcell = num2cell(zeros(size(InMat,1),3+size(varargin,2)/2));
InMatcell(1:size(InMat,1),1:size(InMat,2)) = num2cell(InMat);
FID = fopen(FileName,'w');
%hlavicka csv suboru %csv file header
if (size(varargin,2) > 0)
    fprintf(FID,'Latitude,Longitude,%s,',HType);
    fprintf(FID,'%s,', varargin{1:2:end-2});
    fprintf(FID,'%s\n', varargin{end-1});
else
    fprintf(FID,'Latitude,Longitude,%s\n',HType);
end
%pripravenie prvku na zapis do suboru %preparing cell array for writing to file
for i = 2:2:size(varargin,2)
    if (ArgType(3+i/2) == 1)
        for k = 1:max(size(varargin{i}))
            varargin{i}{k} = num2str(varargin{i}{k});
        end
    end
    if (max(size(varargin{i})) == 1)
        InMatcell(:,i/2+3) = varargin{i};
    else
        InMatcell(1:max(size(varargin{i})),i/2+3) = varargin{i}(:);
        InMatcell(max(size(varargin{i}))+1:end,i/2+3) = {''};
    end
end
%zapis hodnot do csv suboru %writing values into csv file
for j = 1:size(InMatcell,1)
    for i = 1:size(InMatcell,2)
        if (i == size(InMatcell,2) && ArgType(i) == 0)
            fprintf(FID,'%f\n', InMatcell{j,i});
        elseif (i == size(InMatcell,2) && ArgType(i) == 1)
            fprintf(FID,'%s\n', InMatcell{j,i});
        elseif (i == size(InMatcell,2) && ArgType(i) == 2)
            fprintf(FID,'%d\n', InMatcell{j,i});
        elseif (ArgType(i) == 0)
            fprintf(FID,'%f,', InMatcell{j,i});
        elseif (ArgType(i) == 1)
            fprintf(FID,'%s,', InMatcell{j,i});
        elseif (ArgType(i) == 2)
            fprintf(FID,'%d,', InMatcell{j,i});
        end
    end
end
%%
%ukoncenie zapisu %end of writing to file
OutCell = InMatcell;
fclose(FID);
end