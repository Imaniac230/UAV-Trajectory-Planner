%%
%init
clear
clc
addpath('..\src_trajectory_planner\miscellaneous\', '..\src_trajectory_planner\horizontal_trajectory\', '..\src_trajectory_planner\vertical_trajectory\')
FID = 1;
%%
%parameter and path definitions
scriptstart = tic;

CSV_Plan_Data_Path(1) = {'..\data\csv_trajectories\planned_trajectory_name.csv'};
CSV_Plan_Data_Path(2) = {'..\data\csv_trajectories\planned_trajectory_name.csv'};
CSV_DJI_Data_Path = '..\data\flight_data_from_DJI\FLYxxx.csv';
Elevation_Model_Path = '..\data\terrain_models\dem_name.xyz';
%%
fprintf(FID,'%s Loading variables ...\n',char(datetime('now')));
tic
%parameters preparation
%terrain model
TerrMod = load(Elevation_Model_Path);
TerrMod(:,4) = TerrMod(:,1);
TerrMod(:,1) = TerrMod(:,2);
TerrMod(:,2) = TerrMod(:,4);
TerrMod(:,4) = [];
TerrMod(:,3) = TerrMod(:,3) - 44.831;
%planned trajectory
Planned_Trajectories = cell((sum(size(CSV_Plan_Data_Path))-1),1);
for i = 1:(sum(size(CSV_Plan_Data_Path))-1)
    PlanTrj = csvread(CSV_Plan_Data_Path{i},1);
    PlanTrj = PlanTrj(:,1:3);
    Planned_Trajectories(i) = {PlanTrj};
end
%real trajectory
FlightData = readtable(CSV_DJI_Data_Path);
RealTrj = zeros(size(FlightData,1),3);
RealTrj(:,2) = FlightData{:,'GPS_0__Long'};
RealTrj(:,1) = FlightData{:,'GPS_0__Lat'};
RealTrj(:,3) = FlightData{:,'GPS_0__heightMSL'};
%clear repeating points and NaN values in real trajectory
RealTrj = unique(RealTrj,'rows','stable');
RealTrj(any(isnan(RealTrj), 2), :) = [];
%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%IF THE THE DATA CONTAINS MORE THAN ONE FLIGH MISSION PLEASE SEPARATE THEM MANUALLY HERE

Real_Trajectories = {RealTrj(1:3100,:) ; RealTrj(3100:end,:)};

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

fprintf(FID,'%s Done, time elapsed: %fs\n\n',char(datetime('now')),toc);
%%
%function call
fprintf(FID,'%s Initiating function "analyzeFlight_DJI", %d flight missions given ...\n',char(datetime('now')),sum(size(Real_Trajectories))-1);

RealTrjOut = cell((sum(size(Real_Trajectories))-1),1);
RealTrjDistOut = cell((sum(size(Real_Trajectories))-1),1);
TerrTrjOut = cell((sum(size(Real_Trajectories))-1),1);
TerrTrjDistOut = cell((sum(size(Real_Trajectories))-1),1);

for i=1:(sum(size(Real_Trajectories))-1)
    [RealTrjOut{i},RealTrjDistOut{i},TerrTrjOut{i},TerrTrjDistOut{i}] = analyzeFlight_DJI(Real_Trajectories{i},Planned_Trajectories{i},TerrMod);
end
fprintf(FID,'%s Done, time elapsed: %fs\n\n',char(datetime('now')),toc);
%%
%cleanup
fprintf(FID,'The script took %f seconds\n',toc(scriptstart));
rmpath('..\src_release\miscellaneous\', '..\src_release\horizontal_trajectory\', '..\src_release\vertical_trajectory\')