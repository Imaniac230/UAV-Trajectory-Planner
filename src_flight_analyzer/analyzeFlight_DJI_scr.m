%%
%init
clear
clc
addpath(genpath('..\src_trajectory_planner\'))
FID = 1;
%%
%parameter and path definitions
scriptstart = tic;

Use_GPS_Values = false;

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
if (Use_GPS_Values)
    RealTrj(:,2) = FlightData{:,'GPS_0__Long'};
    RealTrj(:,1) = FlightData{:,'GPS_0__Lat'};
    RealTrj(:,3) = FlightData{:,'GPS_0__heightMSL'};
else
    RealTrj(:,1) = FlightData{:,'IMU_ATTI_0__Latitude'};
    RealTrj(:,2) = FlightData{:,'IMU_ATTI_0__Longitude'};
    RealTrj(:,3) = FlightData{:,'IMU_ATTI_0__barometer_Smooth'};

    Motors(:,1) = FlightData{:,'MotorCtrl_PWM_RFront'};
    Motors(:,2) = FlightData{:,'MotorCtrl_PWM_LFront'};
    Motors(:,3) = FlightData{:,'MotorCtrl_PWM_RBack'};
    Motors(:,4) = FlightData{:,'MotorCtrl_PWM_LBack'};
    MotoStart = min([find(Motors(:,1),1,'first') find(Motors(:,2),1,'first') find(Motors(:,3),1,'first') find(Motors(:,4),1,'first')]);
    TerrStart = trjmap2dem(RealTrj(MotoStart,:),TerrMod);

    RealTrj(:,3) = RealTrj(:,3) - RealTrj(MotoStart,3) + TerrStart(3);
end
%clear repeating points and NaN values in real trajectory
RealTrj = unique(RealTrj,'rows','stable');
RealTrj(any(isnan(RealTrj), 2), :) = [];
%distance travelled from flight data
DistTrav = FlightData{:,'IMU_ATTI_0__distanceTravelled'};
%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%IF THE THE DATA CONTAINS MORE THAN ONE FLIGH MISSION PLEASE SEPARATE THEM MANUALLY HERE

Real_Trajectories = {RealTrj(1:6200,:) ; RealTrj(6200:end,:)};
External_Distance = {DistTrav(1:6200,:) ; DistTrav(6200:end,:)};

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

fprintf(FID,'%s Done, time elapsed: %d min. %f s\n\n',char(datetime('now')),floor(toc/60),(toc/60-floor(toc/60))*60);
%%
%function call
fprintf(FID,'%s Initiating function "analyzeFlight_DJI", %d flight missions given ...\n',char(datetime('now')),sum(size(Real_Trajectories))-1);

RealTrjOut = cell((sum(size(Real_Trajectories))-1),1);
RealTrjDistOut = cell((sum(size(Real_Trajectories))-1),1);
TerrTrjOut = cell((sum(size(Real_Trajectories))-1),1);
TerrTrjDistOut = cell((sum(size(Real_Trajectories))-1),1);
IdxOut = cell((sum(size(Real_Trajectories))-1),1);

for i=1:(sum(size(Real_Trajectories))-1)
    [RealTrjOut{i},RealTrjDistOut{i},TerrTrjOut{i},TerrTrjDistOut{i},IdxOut{i}] = analyzeFlight_DJI(Real_Trajectories{i},Planned_Trajectories{i},TerrMod,'ExternDist',External_Distance{i});
end
fprintf(FID,'%s Done, time elapsed: %d min. %f s\n\n',char(datetime('now')),floor(toc/60),(toc/60-floor(toc/60))*60);
%%
%cleanup
fprintf(FID,'The script took %d minutes %f seconds\n',floor(toc(scriptstart)/60),(toc(scriptstart)/60-floor(toc(scriptstart)/60))*60);
rmpath(genpath('..\src_trajectory_planner\'))