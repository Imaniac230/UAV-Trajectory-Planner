%%%%%%%%%%%%%%%%%%%%%%%%%%%% MODOFICATIONS BEYOND THIS POINT AT YOUR OWN RISK! %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%
%file path definitions
logfilepath = Log_Name;
polfilepath = Input_Polygon;
shfilepath = Shapefile_Name;
modfilepath = Digital_Elevation_Model;
csvfilepath = CSV_Name;
if (exist('Custom_Trajectory_Lines','var'))
    customlinefilepath = Custom_Trajectory_Lines;
else
    customlinefilepath = '';
end
if (exist('Custom_Trajectory_Points','var'))
    custompointfilepath = Custom_Trajectory_Points;
else
    custompointfilepath = '';
end

%verify logical flag values
if (sum(size(Interactive_UI))-1 > 5)
    error('Interactive_UI parameter must not be larger than 5 elements.')
end
if ((sum(size(Interactive_UI)) == 2) && ~islogical(Interactive_UI))
    error('Interactive_UI parameter must be a logical value.')
elseif (sum(size(Interactive_UI)) > 2)
    for i = 1:sum(size(Interactive_UI))-1
        if ((Interactive_UI(i) ~= 1) && (Interactive_UI(i) ~= 0))
            error('Interactive_UI(%d) parameter must be a logical value.',i)
        end
    end
    Interactive_UItmp = zeros(1,sum(size(Interactive_UI))-1-i);
    Interactive_UI = [Interactive_UI Interactive_UItmp];
else
    Interactive_UItmp = zeros(1,4);
    Interactive_UI = [Interactive_UI Interactive_UItmp];
end
if (~islogical(Log_To_File) || (sum(size(Log_To_File)) > 2))
    error('Log_To_File parameter must be a logical value.')
end
if (~islogical(Create_Shapefile) || (sum(size(Create_Shapefile)) > 2))
    error('Create_Shapefile parameter must be a logical value.')
end
if (~islogical(Create_CSV_File) || (sum(size(Create_CSV_File)) > 2))
    error('Create_CSV_File parameter must be a logical value.')
end
if (~islogical(Fill_Entire_Polygon) || (sum(size(Fill_Entire_Polygon)) > 2))
    error('Fill_Entire_Polygon parameter must be a logical value.')
end
if (~islogical(Variable_Trajectory_Segmentation) || (sum(size(Variable_Trajectory_Segmentation)) > 2))
    error('Variable_Trajectory_Segmentation parameter must be a logical value.')
end
if (~islogical(Convert_Trajectory_To_Orthometric_Height) || (sum(size(Convert_Trajectory_To_Orthometric_Height)) > 2))
    error('Convert_Trajectory_To_Orthometric_Height parameter must be a logical value.')
end
if (~islogical(Verify_UAV_Ascent_Limit) || (sum(size(Verify_UAV_Ascent_Limit)) > 2))
    error('Verify_UAV_Ascent_Limit parameter must be a logical value.')
end
if (~islogical(Profile_2D_Plot) || (sum(size(Profile_2D_Plot)) > 2))
    error('Profile_2D_Plot parameter must be a logical value.')
end
if (~islogical(Full_3D_Plot) || (sum(size(Full_3D_Plot)) > 2))
    error('Full_3D_Plot parameter must be a logical value.')
end
if (~islogical(Plots_In_Local_Frame) || (sum(size(Plots_In_Local_Frame)) > 2))
    error('Plots_In_Local_Frame parameter must be a logical value.')
end
if (~islogical(Use_Existing_Terrain_3DPlot) || (sum(size(Use_Existing_Terrain_3DPlot)) > 2))
    error('Use_Existing_Terrain_3DPlot parameter must be a logical value.')
end
if (~islogical(Generated_Trajectory_Line) || (sum(size(Generated_Trajectory_Line)) > 2))
    error('Generated_Trajectory_Line parameter must be a logical value.')
end
if (~islogical(Segmented_Trajectory_Points) || (sum(size(Segmented_Trajectory_Points)) > 2))
    error('Segmented_Trajectory_Points parameter must be a logical value.')
end
if (~islogical(Digital_Elevation_Model_Points) || (sum(size(Digital_Elevation_Model_Points)) > 2))
    error('Digital_Elevation_Model_Points parameter must be a logical value.')
end

%verify custom override inputs
if (~strcmp(customlinefilepath,'') && ~strcmp(custompointfilepath,''))
    error('Only one override input can be defined at a time. If you do not wish to use an override input please leave it as an empty string ('''') or comment it out.')
end
if (~strcmp(customlinefilepath,''))
    inputfilepath = customlinefilepath;
elseif (~strcmp(custompointfilepath,''))
    inputfilepath = custompointfilepath;
else
    inputfilepath = polfilepath;
end

%check for existing terrain 3D plot
if (Full_3D_Plot && Use_Existing_Terrain_3DPlot)
    if (~exist('TERRAINPLOT3DP','var'))
        warning('UseExisting3DPlot:PlotNotFound','"Use_Existing_Terrain_3DPlot" parameter was set to ''true'' but no existing terrain 3D plot was found. Please first run this script with "Use_Existing_Terrain_3DPlot" set to ''false'' to generate a plot or provide one to the Matlab Workspace with the name ''TERRAINPLOT3DP''.')
        [baseName, folder] = uigetfile('*.mat','Select an existing terrain 3D plot:','..\data\terrain_plots');
        TERRAINPLOT3Dpath = fullfile(folder, baseName);
        [~,~,ext] = fileparts(TERRAINPLOT3Dpath);
        if (~strcmp(ext,'.mat'))
            error('Incorrect file extension for existing 3D plot. Please choose a *.mat file.')
        end
        tp = load(TERRAINPLOT3Dpath);
        tp = struct2cell(tp);
        TERRAINPLOT3DP = tp{1};
    end
    
    if (~exist('TERRAINPLOT3DP_Orthometric','var'))
        dlgmsg = 'Is the height parameter of the loaded terrain 3D plot specified as Geodetic or Orthometric?';
        dlgtitle = 'Choose height specification';
        opts = struct('Default','Geodetic','Interpreter','none');
        selection = questdlg(dlgmsg,dlgtitle,'Geodetic','Orthometric','Cancel',opts);
        switch selection
            case 'Geodetic'
                TERRAINPLOT3DP_Orthometric = false;
            case 'Orthometric'
                TERRAINPLOT3DP_Orthometric = true;
            otherwise
                error('Imported 3D plot height not specified.')
        end
    end
end

%interactive UI file selection
if (strcmp(customlinefilepath,'') && strcmp(custompointfilepath,''))
    if (Interactive_UI(1))
        [baseName, folder] = uigetfile('*.txt;*.shp','Select the input polygon:','..\data\input_polygons');
        polfilepath = fullfile(folder, baseName);
        inputfilepath = polfilepath;
    end
end
if (Interactive_UI(2))
    [baseName, folder] = uigetfile('*.xyz','Select the input elevation model:','..\data\terrain_models');
    modfilepath = fullfile(folder, baseName);
end

scriptstart = tic;

%log file creation
clk = fix(clock);
clkn = sprintf('%d-%d-%d-T-%02d-%02d-%02d_', clk(3),clk(2),clk(1),clk(4),clk(5),clk(6));
if (Log_To_File)
    [~,name] = fileparts(inputfilepath);
    name = strcat(clkn,name,'.log');
    if (Interactive_UI(4))
        [baseName, folder] = uiputfile(name,'Save log to:','..\data\logs');
        baseName = strcat(clkn,baseName);
        logfilepath = fullfile(folder, baseName);
    elseif (strcmp(logfilepath,''))
        logfilepath = strcat(logfilepath,'..\data\logs\',name);
    elseif (strcmp(logfilepath,'..\data\logs\'))
        logfilepath = strcat(logfilepath,name);
    else
        [path,name,ext] = fileparts(logfilepath);
        name = strcat(clkn,name,ext);
        logfilepath = fullfile(path,name);
    end
    [~,~,ext] = fileparts(logfilepath);
    if (strcmp(ext,''))
        logfilepath = strcat(logfilepath,'.log');
    end
    diary(logfilepath)
end

%displaying configured parameters and other info
FID = 1;
try
    fprintf(FID,'%s Script initiated with parameters:\n\n\tFocal Length: %.6gmm\n\tCCD Sensor: %.6gx%.6gmm\n\tHeight Above Ground Level: %.6gm\n\tSide Overlap: %.6g%%\n\tCCD Side perpendicular to trajectory: %.6gmm\n\tPolygon Reference Point: %d.\n\tGeoid-Ellipsoid Separation: %.6gm\n', ... 
        char(datetime('now')),Focal_Length,CCD_Sensor_Size(1),CCD_Sensor_Size(2),Height_Above_Ground_Level,Side_Overlap_Factor,CCD_Sensor_Size(CCD_Sensor_Parameter_Option),Polygon_Reference_Point_Option,Geoid_Ellipsoid_Separation);
catch err
    if (strcmp(err.identifier,'MATLAB:badsubscript'))
        error('Invalid CCD_Sensor_Parameter_Option. Input must be eiher 1 or 2.')
    end
end
if (Variable_Trajectory_Segmentation)
    fprintf(FID,'\tHorizontal distance between waypoints: variable (min. %.6gm)\n\n',Minimal_Trajectory_Waypoint_Separation);
else
    fprintf(FID,'\tHorizontal distance between waypoints: %.6gm\n\n',Minimal_Trajectory_Waypoint_Separation);
end

if (strcmp(customlinefilepath,'') && strcmp(custompointfilepath,''))
    if (Trajectory_Line_Separation_Manual)
        fprintf(FID,'%s Manual trajectory line separation set: %.6gm!\n\n',char(datetime('now')),Trajectory_Line_Separation_Manual);
    end
    if (Number_Of_Trajectory_Lines_Manual)
        fprintf(FID,'%s Manual number of trajectory lines set: %d!\n\n',char(datetime('now')),Trajectory_Line_Separation_Manual);
    end
    if (Trajectory_Offset_From_Origin)
        fprintf(FID,'%s Manual trajectory line separation set: %.6gm!\n\n',char(datetime('now')),Trajectory_Line_Separation_Manual);
    end
end

if (~strcmp(customlinefilepath,''))
    fprintf(FID,'%s Used custom line: %s\n',char(datetime('now')),customlinefilepath);
elseif (~strcmp(custompointfilepath,''))
    fprintf(FID,'%s Used custom points: %s\n',char(datetime('now')),custompointfilepath);
else
    fprintf(FID,'%s Used polygon: %s\n',char(datetime('now')),polfilepath);
end
fprintf(FID,'%s Used digital elevation model: %s\n\n',char(datetime('now')),modfilepath);

%function calls
addpath('.\miscellaneous\', '.\horizontal_trajectory\', '.\vertical_trajectory\')

fprintf(FID,'%s Loading input shapes ...\n',char(datetime('now')));
tic
[~,~,ext] = fileparts(inputfilepath);
if (strcmp(ext,'.txt'))
    inputshape = load(inputfilepath);
elseif (strcmp(ext,'.shp'))
    try
        shape = shaperead(inputfilepath);
    catch err
        if (strcmp(err.identifier, 'map:shapefile:failedToOpenSHP'))
            error('Unable to read file ''%s''. No such file or directory.', inputfilepath)
        else
            error(err.message)
        end
    end
    inputshape(:,1) = shape.Y(:);
    inputshape(:,2) = shape.X(:);
    inputshape(any(isnan(inputshape),2),:) = [];
    if (min(inputshape(end,:) == inputshape(1,:)) == 1)
        inputshape(end,:) = [];
    end
else
    error('Incorrect input file extension in ''%s''. Please use a *.txt or *.shp file as input.', inputfilepath)
end
if (~strcmp(customlinefilepath,''))
    outhortraj = inputshape;
elseif (~strcmp(custompointfilepath,''))
    cuttraj = inputshape;
else
    polygon = inputshape;
end
fprintf(FID,'%s Done, time elapsed: %d min. %f s\n\n',char(datetime('now')),floor(toc/60),(toc/60-floor(toc/60))*60);
fprintf(FID,'%s Loading digital elevation model ...\n',char(datetime('now')));
tic
model = load(modfilepath);
modeltmp = model(:,1);
model(:,1) = model(:,2);
model(:,2) = modeltmp;
fprintf(FID,'%s Done, time elapsed: %d min. %f s\n\n',char(datetime('now')),floor(toc/60),(toc/60-floor(toc/60))*60);


if (strcmp(customlinefilepath,'') && strcmp(custompointfilepath,''))
    fprintf(FID,'%s Boundary polygon coordinates in WGS 84 (lat, lon):\n',char(datetime('now')));
    for i = 1:size(polygon,1)
        fprintf(FID,'\n\t%.4f %.4f [°]',polygon(i,1),polygon(i,2));
    end
end
fprintf(FID,'\n\n');

fprintf(FID,'%s Initiating function "trjphotogr2linedist" ...\n',char(datetime('now')));
tic
photooptim_linedist = trjphotogr2linedist(Focal_Length,Height_Above_Ground_Level,CCD_Sensor_Size,Side_Overlap_Factor,CCD_Sensor_Parameter_Option);
fprintf(FID,'%s Done, time elapsed: %d min. %f s\n\n',char(datetime('now')),floor(toc/60),(toc/60-floor(toc/60))*60);
fprintf(FID,'%s Calculated horizontal separation of trajectory lines for %.6g%% side overlap: %.3fm\n\n',char(datetime('now')),Side_Overlap_Factor,photooptim_linedist);

if (strcmp(customlinefilepath,'') && strcmp(custompointfilepath,''))
    fprintf(FID,'%s Initiating function "trjgenhor" ...\n',char(datetime('now')));
    if (Trajectory_Line_Separation_Manual > 0)
        if (Number_Of_Trajectory_Lines_Manual > 0)
            tic
            outhortraj = trjgenhor(polygon,Trajectory_Line_Separation_Manual,Polygon_Reference_Point_Option,Fill_Entire_Polygon,[Number_Of_Trajectory_Lines_Manual Trajectory_Offset_From_Origin]);
        else
            tic
            outhortraj = trjgenhor(polygon,Trajectory_Line_Separation_Manual,Polygon_Reference_Point_Option,Fill_Entire_Polygon);
        end
    else
        if (Number_Of_Trajectory_Lines_Manual > 0)
            tic
            outhortraj = trjgenhor(polygon,photooptim_linedist,Polygon_Reference_Point_Option,Fill_Entire_Polygon,[Number_Of_Trajectory_Lines_Manual Trajectory_Offset_From_Origin]);
        else
            tic
            outhortraj = trjgenhor(polygon,photooptim_linedist,Polygon_Reference_Point_Option,Fill_Entire_Polygon);
        end
    end
    fprintf(FID,'%s Done, time elapsed: %d min. %f s\n\n',char(datetime('now')),floor(toc/60),(toc/60-floor(toc/60))*60);
end
if (strcmp(custompointfilepath,''))
    fprintf(FID,'%s Number of points: %d\n%s Number of coordinates: %d\n',char(datetime('now')),size(outhortraj,1),char(datetime('now')),size(outhortraj,2));
    [trjpdist,~,~,adl,numl] = trjstats(outhortraj);
    fprintf(FID,'%s Average horizontal distance between trajectory lines: %.3fm\n',char(datetime('now')),adl);
    fprintf(FID,'%s Number of trajectory lines: %d\n\n',char(datetime('now')),numl);
end

if (strcmp(custompointfilepath,''))
    if (Variable_Trajectory_Segmentation)
        fprintf(FID,'%s Initiating function "trjcutseghorvar" ...\n',char(datetime('now')));
        if (Perpendicular_Safe_Distance)
            tic
            [cuttraj,nwaypoints,mpwpsepar,grad,modeltraj] = trjcutseghorvar(outhortraj,model,Gradient_Difference_Tolerance,Perpendicular_Safe_Distance);
            fprintf(FID,'%s Done, time elapsed: %d min. %f s\n\n',char(datetime('now')),floor(toc/60),(toc/60-floor(toc/60))*60);
        else
            tic
            [cuttraj,nwaypoints,mpwpsepar,grad,modeltraj] = trjcutseghorvar(outhortraj,model,Gradient_Difference_Tolerance);
            fprintf(FID,'%s Done, time elapsed: %d min. %f s\n\n',char(datetime('now')),floor(toc/60),(toc/60-floor(toc/60))*60);
        end
        [wpdist1,~,adp1] = trjstats(cuttraj);
    else
        fprintf(FID,'%s Initiating function "trjcutseghor" ...\n',char(datetime('now')));
        tic
        [cuttraj,nwaypoints] = trjcutseghor(outhortraj,Minimal_Trajectory_Waypoint_Separation);
        fprintf(FID,'%s Done, time elapsed: %d min. %f s\n\n',char(datetime('now')),floor(toc/60),(toc/60-floor(toc/60))*60);
        fprintf(FID,'%s Number of points: %d\n%s Number of coordinates: %d\n',char(datetime('now')),size(cuttraj,1),char(datetime('now')),size(cuttraj,2));
        [wpdist1,~,adp1,adl,numl] = trjstats(cuttraj);
        fprintf(FID,'%s Average horizontal distance between trajectory lines: %.3fm\n',char(datetime('now')),adl);
        fprintf(FID,'%s Number of trajectory lines: %d\n',char(datetime('now')),numl);
        fprintf(FID,'%s Average horizontal distance between waypoints: %.3fm\n\n',char(datetime('now')),adp1);

        fprintf(FID,'%s Initiating function "trjmap2dem" ...\n',char(datetime('now')));
        tic
        [cuttraj,modeltraj,mpwpsepar] = trjmap2dem(cuttraj,model);
        fprintf(FID,'%s Done, time elapsed: %d min. %f s\n\n',char(datetime('now')),floor(toc/60),(toc/60-floor(toc/60))*60);
    end
else
    fprintf(FID,'%s Number of points: %d\n%s Number of coordinates: %d\n',char(datetime('now')),size(cuttraj,1),char(datetime('now')),size(cuttraj,2));
    [wpdist1,~,adp1,adl,numl] = trjstats(cuttraj);
    fprintf(FID,'%s Average horizontal distance between trajectory lines: %.3fm\n',char(datetime('now')),adl);
    fprintf(FID,'%s Number of trajectory lines: %d\n',char(datetime('now')),numl);
    fprintf(FID,'%s Average horizontal distance between waypoints: %.3fm\n\n',char(datetime('now')),adp1);
    
    fprintf(FID,'%s Initiating function "trjmap2dem" ...\n',char(datetime('now')));
    tic
    [cuttraj,modeltraj,mpwpsepar] = trjmap2dem(cuttraj,model);
    fprintf(FID,'%s Done, time elapsed: %d min. %f s\n\n',char(datetime('now')),floor(toc/60),(toc/60-floor(toc/60))*60);
end
fprintf(FID,'%s Number of points: %d\n%s Number of coordinates: %d\n',char(datetime('now')),size(cuttraj,1),char(datetime('now')),size(cuttraj,2));
[wpdist2,ah,adp2,adl,numl] = trjstats(cuttraj);
fprintf(FID,'%s Average horizontal distance between trajectory lines: %.3fm\n',char(datetime('now')),adl);
fprintf(FID,'%s Number of trajectory lines: %d\n',char(datetime('now')),numl);
fprintf(FID,'%s Average horizontal distance between waypoints: %.3fm\n',char(datetime('now')),adp2);
fprintf(FID,'%s Average height of the terrain above mean sea level: %.3fm\n',char(datetime('now')),ah);
fprintf(FID,'%s Maximal distance between waypoint and mapped digital elevation model point: %.3fm\n\n',char(datetime('now')),max(mpwpsepar));

switch Vertical_Trajectory_Algorithm
    case 1
        fprintf(FID,'%s Initiating function "trjoffsetver" ...\n',char(datetime('now')));
        tic
        [fintraj] = trjoffsetver(cuttraj,Height_Above_Ground_Level);
        fprintf(FID,'%s Done, time elapsed: %d min. %f s\n\n',char(datetime('now')),floor(toc/60),(toc/60-floor(toc/60))*60);
        fprintf(FID,'%s Initiating function "trjfilthor" ...\n',char(datetime('now')));
        tic
        [fintraj] = trjfilthor(fintraj,Minimal_Trajectory_Waypoint_Separation);
        fprintf(FID,'%s Done, time elapsed: %d min. %f s\n\n',char(datetime('now')),floor(toc/60),(toc/60-floor(toc/60))*60);
        fprintf(FID,'%s Number of points: %d\n%s Number of coordinates: %d\n',char(datetime('now')),size(fintraj,1),char(datetime('now')),size(fintraj,2));
        [wpdist,ah1,adp] = trjstats(fintraj);
        fprintf(FID,'%s Average horizontal distance between waypoints: %.3fm\n',char(datetime('now')),adp);
        fprintf(FID,'%s Average height of the trajectory above mean sea level: %.3fm\n',char(datetime('now')),ah1);
        fprintf(FID,'%s Average height above ground level: %.3fm\n',char(datetime('now')),ah1-ah);
        [wptrr,awptrr] = trjterrfilt(fintraj,model);
        fprintf(FID,'%s Minimal distance from terrain: %.3fm\n\n',char(datetime('now')),min(wptrr));      
    case 2
        fprintf(FID,'%s Initiating function "trjradialdistver" ...\n',char(datetime('now')));
        tic
        [fintraj] = trjradialdistver(cuttraj,Height_Above_Ground_Level,model,Minimal_Trajectory_Waypoint_Separation);
        fprintf(FID,'%s Done, time elapsed: %d min. %f s\n\n',char(datetime('now')),floor(toc/60),(toc/60-floor(toc/60))*60);
        [wpdist,ah1,adp] = trjstats(fintraj);
        [wptrr,awptrr] = trjterrfilt(fintraj,model);
        if (min(wptrr) < (Height_Above_Ground_Level))
            [fintraj,wptrr] = trjterrfilt(fintraj,model,Height_Above_Ground_Level);
            [wpdist,ah1,adp] = trjstats(fintraj);
            fprintf(FID,'%s Number of points: %d\n%s Number of coordinates: %d\n',char(datetime('now')),size(fintraj,1),char(datetime('now')),size(fintraj,2));
            fprintf(FID,'%s Average horizontal distance between waypoints: %.3fm\n',char(datetime('now')),adp);
            fprintf(FID,'%s Average height of the trajectory above mean sea level: %.3fm\n',char(datetime('now')),ah1);
            fprintf(FID,'%s Average height above ground level: %.3fm\n',char(datetime('now')),ah1-ah);
            fprintf(FID,'%s Minimal distance from terrain: %.3fm\n\n',char(datetime('now')),min(wptrr));
        else
            fprintf(FID,'%s Number of points: %d\n%s Number of coordinates: %d\n',char(datetime('now')),size(fintraj,1),char(datetime('now')),size(fintraj,2));
            fprintf(FID,'%s Average horizontal distance between waypoints: %.3fm\n',char(datetime('now')),adp);
            fprintf(FID,'%s Average height of the trajectory above mean sea level: %.3fm\n',char(datetime('now')),ah1);
            fprintf(FID,'%s Average height above ground level: %.3fm\n',char(datetime('now')),ah1-ah);
            fprintf(FID,'%s Minimal distance from terrain: %.3fm\n\n',char(datetime('now')),min(wptrr));
        end
    case 0
        adp = adp1;
        wpdist = wpdist1;
        fintraj = cuttraj;
        warning('VerticalAlgorithm:NotUsed','Trajectory height was not modified.')
    otherwise
        error('Invalid Vertical_Trajectory_Algorithm! Please specify one of the available algorithms to be used for vertical terrain copying.')
end

if (((adp1 ~= adp2) + (adp ~= adp1) + (adp ~= adp2)) ~= 0)
    fprintf(FID,'%s The initial positions of waypoints have been modified!\n\n',char(datetime('now')));
end

%conversion to orthometric height
if (Convert_Trajectory_To_Orthometric_Height)
    fintraj(:,3) = fintraj(:,3) - Geoid_Ellipsoid_Separation;
    modeltraj(:,3) = modeltraj(:,3) - Geoid_Ellipsoid_Separation;
    model(:,3) = model(:,3) - Geoid_Ellipsoid_Separation;
end

%UAV limitations verification and trajectory correction
if (Verify_UAV_Ascent_Limit)
    fintraj = trjascentlim(fintraj,Speed_Column_For_CSV_File,Maximum_UAV_Vertical_Speed);
end

%shapefile creation
if (Create_Shapefile)
    if (Generated_Trajectory_Line && strcmp(custompointfilepath,''))
        if (Interactive_UI(3))
            [~,name] = fileparts(inputfilepath);
            name = strcat(name,'.shp');
            [baseName, folder] = uiputfile(name,'Save shapefile to:','..\data\output_lines');
            shfilepath = fullfile(folder, baseName);
        end
        shapecreate(outhortraj,Geometry,id,shfilepath);
        x = textscan(shfilepath,'%s','Delimiter','\');
        fprintf(FID,'%s Shapefile "%s" created!\n\n',char(datetime('now')),char(x{1}(size(x{1},1))));
    elseif (Segmented_Trajectory_Points)
        if (Interactive_UI(3))
            [~,name] = fileparts(inputfilepath);
            name = strcat(name,'.shp');
            [baseName, folder] = uiputfile(name,'Save shapefile to:','..\data\output_points');
            shfilepath = fullfile(folder, baseName);
        end
        shapecreate(cuttraj,Geometry,id,shfilepath);
        x = textscan(shfilepath,'%s','Delimiter','\');
        fprintf(FID,'%s Shapefile "%s" created!\n\n',char(datetime('now')),char(x{1}(size(x{1},1))));
    elseif (Digital_Elevation_Model_Points)
        if (Interactive_UI(3))
            [~,name] = fileparts(inputfilepath);
            name = strcat(name,'.shp');
            [baseName, folder] = uiputfile(name,'Save shapefile to:','..\data\output_points');
            shfilepath = fullfile(folder, baseName);
        end
        shapecreate(modeltraj,Geometry,id,shfilepath);
        x = textscan(shfilepath,'%s','Delimiter','\');
        fprintf(FID,'%s Shapefile "%s" created!\n\n',char(datetime('now')),char(x{1}(size(x{1},1))));
    else
        warning('CreateShape:NoShapeDefined','No defined shapefile to generate!')
    end 
end

%csv file name
if (Create_CSV_File)
    [~,name] = fileparts(inputfilepath);
    name = strcat(clkn,name,'.csv');
    if (Interactive_UI(5))
        [baseName, folder] = uiputfile(name,'Save csv to:','..\data\csv_trajectories');
        baseName = strcat(clkn,baseName);
        csvfilepath = fullfile(folder, baseName);
    elseif (strcmp(csvfilepath,''))
        csvfilepath = strcat(csvfilepath,'..\data\csv_trajectories\',name);
    elseif (strcmp(csvfilepath,'..\data\csv_trajectories\'))
        csvfilepath = strcat(csvfilepath,name);
    else
        [path,name,ext] = fileparts(csvfilepath);
        name = strcat(clkn,name,ext);
        csvfilepath = fullfile(path,name);
    end
    [~,~,ext] = fileparts(csvfilepath);
    if (strcmp(ext,''))
        csvfilepath = strcat(csvfilepath,'.csv');
    end
    if (isnumeric(Speed_Column_For_CSV_File))
        speedcmpval = Speed_Column_For_CSV_File;
    else
        speedcmpval = cell2mat(Speed_Column_For_CSV_File);
    end
    if (speedcmpval > 0)
        trj2csv_ugcs(fintraj,'AltitudeAMSL',csvfilepath,'Speed',Speed_Column_For_CSV_File);
    else
        trj2csv_ugcs(fintraj,'AltitudeAMSL',csvfilepath);
    end
    fprintf(FID,'%s CSV file "%s" created!\n\n',char(datetime('now')),csvfilepath);
end
%%
%setting default values of undefined parameters for plots
if (~strcmp(customlinefilepath,'') || ~strcmp(custompointfilepath,''))
    polygon = [fintraj(1,1:2);fintraj(1,1:2);fintraj(1,1:2);fintraj(1,1:2)];
end
if (~strcmp(custompointfilepath,''))
    outhortraj = fintraj(1,1:2);
end
%%
%parameters for 2D plots
fprintf(FID,'%s Calculating parameters for 2D plots ...\n',char(datetime('now')));
tic
if (Plots_In_Local_Frame)
    [polygonL(:,2),polygonL(:,1)] = geodetic2enu(polygon(:,1),polygon(:,2),0,polygon(Polygon_Reference_Point_Option,1),polygon(Polygon_Reference_Point_Option,2),0,wgs84Ellipsoid);
    [outhortrajL(:,2),outhortrajL(:,1)] = geodetic2enu(outhortraj(:,1),outhortraj(:,2),0,polygon(Polygon_Reference_Point_Option,1),polygon(Polygon_Reference_Point_Option,2),0,wgs84Ellipsoid);
    [cuttrajL(:,2),cuttrajL(:,1)] = geodetic2enu(cuttraj(:,1),cuttraj(:,2),0,polygon(Polygon_Reference_Point_Option,1),polygon(Polygon_Reference_Point_Option,2),0,wgs84Ellipsoid);
    [modeltrajL(:,2),modeltrajL(:,1)] = geodetic2enu(modeltraj(:,1),modeltraj(:,2),0,polygon(Polygon_Reference_Point_Option,1),polygon(Polygon_Reference_Point_Option,2),0,wgs84Ellipsoid);
    [fintrajL(:,2),fintrajL(:,1)] = geodetic2enu(fintraj(:,1),fintraj(:,2),0,polygon(Polygon_Reference_Point_Option,1),polygon(Polygon_Reference_Point_Option,2),0,wgs84Ellipsoid);
    X0 = polygonL(:,2)';
    X0 = [X0, polygonL(1,2)];
    Y0 = polygonL(:,1)';
    Y0 = [Y0, polygonL(1,1)];
    X1 = outhortrajL(:,2)';
    Y1 = outhortrajL(:,1)';
    X2 = cuttrajL(:,2)';
    Y2 = cuttrajL(:,1)';
    X3 = modeltrajL(:,2)';
    Y3 = modeltrajL(:,1)';
    X4 = fintrajL(:,2)';
    Y4 = fintrajL(:,1)';
else
    X0 = polygon(:,2)';
    X0 = [X0, polygon(1,2)];
    Y0 = polygon(:,1)';
    Y0 = [Y0, polygon(1,1)];
    X1 = outhortraj(:,2)';
    Y1 = outhortraj(:,1)';
    X2 = cuttraj(:,2)';
    Y2 = cuttraj(:,1)';
    X3 = modeltraj(:,2)';
    Y3 = modeltraj(:,1)';
    X4 = fintraj(:,2)';
    Y4 = fintraj(:,1)';
end
if (Profile_2D_Plot)
    [TERRAINPLOT2D] = trjcutseghor(fintraj,Terrain_2D_3D_Point_Separation);
    lastwarn('');
    [TERRAINPLOT2D] = trjmap2dem(TERRAINPLOT2D,model);
    [~,lastw] = lastwarn;
    if (strcmp(lastw,'MapToDEM:OutOfBounds'))
        warning('Create2DPlot:BadData','Terrain profile 2D plot may contain incorrect data.')
    end
    [terrdist] = trjstats(TERRAINPLOT2D);
    X5 = zeros(1,size(TERRAINPLOT2D,1));
    for i=2:size(X5,2)
    X5(i) = sum(terrdist(1:i-1));
    end
    Y5 = TERRAINPLOT2D(:,3);
    X6 = zeros(1,size(fintraj,1));
    for i=2:size(X6,2)
    X6(i) = sum(wpdist(1:i-1));
    end
    Y6 = fintraj(:,3);
end
fprintf(FID,'%s Done, time elapsed: %d min. %f s\n\n',char(datetime('now')),floor(toc/60),(toc/60-floor(toc/60))*60);

if (Full_3D_Plot)
    %parameters for 3D plots
    fprintf(FID,'%s Calculating parameters for 3D plots ...\n',char(datetime('now')));
    tic
    if (~Use_Existing_Terrain_3DPlot)
        if (~strcmp(custompointfilepath,'') || ~strcmp(customlinefilepath,''))
            error('Can not create a 3D terrain plot for custom override input. Please define a polygon area to fill or provide a pre-existing 3D terrain plot to the Matlab Workspace with the name ''TERRAINPLOT3DP''.')
        else
            [TERRAINPLOT3D] = trjgenhor(polygon,Terrain_3D_Line_Separation,1,true);
        end
        [TERRAINPLOT3D] = trjcutseghor(TERRAINPLOT3D,Terrain_2D_3D_Point_Separation);
        lastwarn('');
        [~,TERRAINPLOT3D] = trjmap2dem(TERRAINPLOT3D,model);
        [~,lastw] = lastwarn;
        if (strcmp(lastw,'MapToDEM:OutOfBounds'))
            warning('Create3DPlot:BadData','Full 3D terrain plot may contain incorrect data.')
        end
        TERRAINPLOT3DP = TERRAINPLOT3D;
        if (Convert_Trajectory_To_Orthometric_Height)
            TERRAINPLOT3DP_Orthometric = true;
        else
            TERRAINPLOT3DP_Orthometric = false;
        end
    end
    if (Plots_In_Local_Frame)
        %in local frame
        polygon_original = polygon;
        fintraj_original = fintraj;
        [polygon(:,2),polygon(:,1)] = geodetic2enu(polygon(:,1),polygon(:,2),0,TERRAINPLOT3DP(1,1),TERRAINPLOT3DP(1,2),0,wgs84Ellipsoid);
        [fintraj(:,2),fintraj(:,1)] = geodetic2enu(fintraj(:,1),fintraj(:,2),0,TERRAINPLOT3DP(1,1),TERRAINPLOT3DP(1,2),0,wgs84Ellipsoid);
        fintraj(:,3) = fintraj(:,3);
        TERRAINPLOT3DL = [];
        [TERRAINPLOT3DL(:,2),TERRAINPLOT3DL(:,1)] = geodetic2enu(TERRAINPLOT3DP(:,1),TERRAINPLOT3DP(:,2),0,TERRAINPLOT3DP(1,1),TERRAINPLOT3DP(1,2),0,wgs84Ellipsoid);
        TERRAINPLOT3DL(:,3) = TERRAINPLOT3DP(:,3);
        TERRAINPLOT3D = TERRAINPLOT3DL;
    else
        TERRAINPLOT3D = TERRAINPLOT3DP;
    end
    if (Use_Existing_Terrain_3DPlot)
        if (~Convert_Trajectory_To_Orthometric_Height && TERRAINPLOT3DP_Orthometric)
            TERRAINPLOT3D(:,3) = TERRAINPLOT3D(:,3) + Geoid_Ellipsoid_Separation;
        elseif (Convert_Trajectory_To_Orthometric_Height && ~TERRAINPLOT3DP_Orthometric)
            TERRAINPLOT3D(:,3) = TERRAINPLOT3D(:,3) - Geoid_Ellipsoid_Separation;
        end
    end
    fprintf(FID,'%s Done, time elapsed: %d min. %f s\n\n',char(datetime('now')),floor(toc/60),(toc/60-floor(toc/60))*60);
end

%2D plots
fprintf(FID,'%s Initiating 2D plots ...\n',char(datetime('now')));
tic
figure('Name','UAV Trajectory Planner')
subplot(2,2,1)
grid on
hold on
plot(X0,Y0,'o--r')
plot(X1,Y1,':xk','LineWidth',1.45)
plot(X1(1),Y1(1),'dg','MarkerSize',10,'LineWidth',1.5)
title('Original polygon with generated trajectory')
if (Plots_In_Local_Frame)
    ylabel('Y [m]')
    xlabel('X [m]')
else
    ylabel('N (latitude) [°]')
    xlabel('E (longitude) [°]')
end
legend('polygon','trajectory','starting point','Location','best')
subplot(2,2,2)
grid on
hold on
grid minor
plot(X0,Y0,'o--r')
plot(X2,Y2,':xb','LineWidth',1.45)
plot(X2(1),Y2(1),'dg','MarkerSize',10,'LineWidth',1.5)
title('Original polygon with trajectory cut into initial segments')
if (Plots_In_Local_Frame)
    ylabel('Y [m]')
    xlabel('X [m]')
else
    ylabel('N (latitude) [°]')
    xlabel('E (longitude) [°]')
end
legend('polygon','trajectory','starting point','Location','best')
subplot(2,2,3)
grid on
hold on
grid minor
plot(X3,Y3,'Or')
plot(X2,Y2,'xb')
title('Digital elevation model points mapped into trajectory')
if (Plots_In_Local_Frame)
    ylabel('Y [m]')
    xlabel('X [m]')
else
    ylabel('N (latitude) [°]')
    xlabel('E (longitude) [°]')
end
legend('model','trajectory','Location','best')
subplot(2,2,4)
grid on
hold on
grid minor
plot(X0,Y0,'o--r')
plot(X4,Y4,':xb','LineWidth',1.45)
plot(X2(1),Y2(1),'dg','MarkerSize',10,'LineWidth',1.5)
title('Original polygon with final modified trajectory')
if (Plots_In_Local_Frame)
    ylabel('Y [m]')
    xlabel('X [m]')
else
    ylabel('N (latitude) [°]')
    xlabel('E (longitude) [°]')
end
legend('polygon','trajectory','starting point','Location','best')
if (Profile_2D_Plot)
    figure('Name','UAV Trajectory Planner')
    subplot(2,1,1)
    grid on
    hold on
    grid minor
    plot(X5,Y5,'-r')
    plot(X6,Y6,':b+')
    title(sprintf('Terrain height profile with flight height agl = %.3f m', Height_Above_Ground_Level))
    if (Convert_Trajectory_To_Orthometric_Height)    
        ylabel('Orthometric height [m]')
    else
        ylabel('Geodetic height [m]')
    end
    xlabel('waypoint distance [m]')
    xlim([-10 max(X6)+10])
    ylim([min(TERRAINPLOT2D(:,3))-1 max(fintraj(:,3))+1])
    legend('terrain height (gl)','trajectory height (agl)','Location','best')
end
fprintf(FID,'%s Done, time elapsed: %d min. %f s\n\n',char(datetime('now')),floor(toc/60),(toc/60-floor(toc/60))*60);

%3D plots
if (Full_3D_Plot)
    fprintf(FID,'%s Initiating simple line 3D plot ...\n',char(datetime('now')));
    tic
    figure('Name','UAV Trajectory Planner 3')
    s=plot3(TERRAINPLOT3D(:,2),TERRAINPLOT3D(:,1),TERRAINPLOT3D(:,3),'-r');
    s1=get(s,'ZData');
    grid on
    grid minor
    hold on
    s=plot3(fintraj(:,2),fintraj(:,1),fintraj(:,3),'x-b');
    s2=get(s,'ZData');
    title(sprintf('Terrain height profile with flight height agl = %.3f m (3D)', Height_Above_Ground_Level))
    if (Convert_Trajectory_To_Orthometric_Height)    
        zlabel('Orthometric height [m]')
    else
        zlabel('Geodetic height [m]')
    end
    if (Plots_In_Local_Frame)
        xlabel('X [m]')
        ylabel('Y [m]')
    else
        xlabel('E (longitude) [°]')
        ylabel('N (latitude) [°]')
    end
    stem3(polygon(:,2),polygon(:,1),ones(size(polygon,1),1)*max(s2(:)),'g','MarkerFaceColor','g','BaseValue',min(s1(:)));
    plot3(fintraj(1,2),fintraj(1,1),fintraj(1,3),'dg','MarkerSize',10,'LineWidth',1.5)
    legend('terrain gl','height agl','polygon','starting point')
    fprintf(FID,'%s Done, time elapsed: %d min. %f s\n\n',char(datetime('now')),floor(toc/60),(toc/60-floor(toc/60))*60);
end
%%
%final cleanup
if (Plots_In_Local_Frame && Full_3D_Plot)
    polygon = polygon_original;
    fintraj = fintraj_original;
end
rmpath('.\miscellaneous\', '.\horizontal_trajectory\', '.\vertical_trajectory\')
fprintf(FID,'%s The whole script took: %d min. %f s\n\n',char(datetime('now')),floor(toc(scriptstart)/60),(toc(scriptstart)/60-floor(toc(scriptstart)/60))*60);
if (Log_To_File)
    diary off
    fprintf('%s Log file "%s" created!\n\n',char(datetime('now')),logfilepath);
end
clearvars -except polygon polygonL model photooptim_linedist outhortraj outhortrajL cuttraj cuttrajL modeltraj modeltrajL mpwpsepar ...
    fintraj fintrajL wpdist wptrr TERRAINPLOT3DP TERRAINPLOT3DL TERRAINPLOT3DP_Orthometric