%%%%%%%%%%%%%%%%%%%%%%%%%%%% USER CONFIGURABLE AREA %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%init
%close all
%clc
clearvars -except TERRAINPLOT3DP TERRAINPLOT3DP_Orthometric
%%
%input parameters

%script flow control parameters
Interactive_UI = [true true true false true]; % specify file names and locations through an interactive window
                                               %[Polygon   Digital_Elevation_Model   Shapefile   Log   CSV_File]
Log_To_File = true; % write consecutive console output to a log file
Create_Shapefile = false; % generate a shapefile for a calculated trajectory
Create_CSV_File = false; % generate a csv file for a calculated trajectory
Speed_Column_For_CSV_File = 2; % specify the optional Speed column for the output CSV file [m/s]
%%
%specify trajectory generation
Fill_Entire_Polygon = true; % fill up the entire area of the polygon with the trajectory
Variable_Trajectory_Segmentation = true; % create trajectory waypoints dynamically based on longitudinal gradient differences of the terrain
Gradient_Difference_Tolerance = 0.15; % tolerance of longitudinal gradient difference beyond which waypoints are created
Perpendicular_Safe_Distance = 0; % perpendicular distance from the trajectory in which gradient differences are detected, in meters [m]
Convert_Trajectory_To_Orthometric_Height = true; % convert final trajectory heights from geodetic to orthometric
Geoid_Ellipsoid_Separation = 44.831; % separation between geoid and ellipsoid for used area of interest in meters [m]
Verify_UAV_Ascent_Limit = true; % verify that trajectory does not exceed UAV vertical ascent limits and correct the trajectory
Maximum_UAV_Vertical_Speed = 3; % maximal vertical speed of the UAV in [m/s]

Vertical_Trajectory_Algorithm = 2; % specify the algorithm to be used for vertical terrain copying
                                   %1->Offset 
                                   %2->Radial Distance
                                   %0->Trajectory height will NOT be modified!
%%
%photogrammetry parameters
Focal_Length = 15; % in milimeters [mm]
CCD_Sensor_Size = [36 24]; % in milimeters [mm]
CCD_Sensor_Parameter_Option = 1; % chooses which CCD sensor paramater (1->a or 2->b) is perpendicular to flight trajectory
Side_Overlap_Factor = 80; % in percent [%]
%%
%trajectory generation parameters
Height_Above_Ground_Level = 7; % in meters [m]
Polygon_Reference_Point_Option = 1; % chooses which polygon vertex is going to be taken as the reference
Minimal_Trajectory_Waypoint_Separation = 1.5; % smallest desired distance between the trajectory waypoints in meters [m]
%explicit parameters for manual trajectory generation control
%if these parameters are set to 0 they will NOT be used
Trajectory_Line_Separation_Manual = 0; % distance between trajectory lines in meters [m], if set the Side_Overlap_Factor parameter will be IGNORED!
Number_Of_Trajectory_Lines_Manual = 0; % defines the number of trajectory lines to be generated
Trajectory_Offset_From_Origin = 0; % defines the offset of the origin of the trajectory from the origin of the polygon in meters [m]
%%
%terrain and trajectory plots parameters
Profile_2D_Plot = true; % choose whether a 2D profile plot of the final trajectory should be drawn
Full_3D_Plot = false; % choose whether a complete 3D plot of the final trajectory should be drawn
Plots_In_Local_Frame = false; % all plots will be shown in a local reference frame in meters [m] instead of polar coordinates
Terrain_3D_Line_Separation = 1; % specify the precision with which the terrain 3D plot is to be drawn in meters [m]
Terrain_2D_3D_Point_Separation = 1; % specify the precision with which the terrain 2D and 3D plot is to be drawn in meters [m]
Use_Existing_Terrain_3DPlot = false; % use a previously generated terrain 3D plot
%%
%parameters of the optional shapefile for a calculated trajectory
Generated_Trajectory_Line = true; % generate lines for planned trajectory
Segmented_Trajectory_Points = true; % generate points of the segmented trajectory
Digital_Elevation_Model_Points = true; % generate points mapped to trajectory from elevation model
Geometry = 'Line'; % geometry type of the shapefile
id = 0; % id of the shape within the shapefile
%%
%specify file names and locations manually
% if the Interactive_UI option for a given parameter is enabled, the corresponding value below will be IGNORED!
Input_Polygon = '..\data\input_polygons\input_polygon_name.shp'; % name and location of the input polygon
Digital_Elevation_Model = '..\data\terrain_models\dem_name.xyz'; % name and location of the digital elevation model
Shapefile_Name = '..\data\output_points\output_shapefile_name.shp'; % name and location for the generated shapefile
CSV_Name = '..\data\csv_trajectories\'; % name and location for the final trajectory csv file, leave empty ('')
                                        % or without a name ('..data\csv_trajectories\') to implicitly name the csv file
                                        % according to the input polygon
Log_Name = '..\data\logs\'; % name and location for the created log file, leave empty ('') or without a name ('..data\logs\')
                            % to implicitly name the log file according to the input polygon
%%
%override the process with custom trajectory line or waypoint inputs
% if an override input is specified some of the previous planning parameters might be IGNORED as a result!
% WARNING: Only one override input can be specified at a time.
%Custom_Trajectory_Lines = '..\data\input_lines\xxx.shp'; % name and location of a custom trajectory line input
%Custom_Trajectory_Points = '..\data\input_points\xxx.shp'; % name and location of a custom trajectory waypoint input
% if you do not wish to use an override parameter leave it empty ('') or comment it out

%%%%%%%%%%%%%%%%%%%%%%%%%%%% USER CONFIGURABLE AREA %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
%script initiation
trajcontrol_scr