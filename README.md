# UAV Trajectory Planner

Source files for the vertical and horizontal UAV trajectory planner and flight analyzer. This project was created as a bachelor's thesis.
Full thesis in native Slovak language can be found in: https://www.vutbr.cz/studenti/zav-prace/detail/119130

# File structure

## src_flight_analyzer

* **analyzeFlight_DJI_scr.m** - control script used for setting input parameters for the analyzer
* **analyzeFlight_DJI.m** - Matlab function source file of the flight analyzer

## src_trajectory_planner

* **trajcontrol_scr_user.m** - control script used for setting input parameters for the trajectory planner
* **trajcontrol_scr.m** - control script used for calling functions and plotting final trajectory graphs in Matlab

### /horizontal_trajectory
 * **trjgenhor.m** - Matlab function source file of the horizontal trajectory generator
 * **trjphotogr2linedist.m** - Matlab function source file of the trajectory line separation for photo side overlap
 * **trjcutseghor.m** - Matlab function source file of the horizontal waypoint generator
 * **trjcutseghorvar.m** - Matlab function source file of the variable horizontal waypoint generator
 * **trjfilthor.m** - Matlab function source file of the horizontal waypoint filter
### /vertical_trajectory
 * **trjmap2dem.m** - Matlab function source file of trajectory and digital elevation model synchronizer
 * **trjoffsetver.m** - Matlab function source file of the vertical offset algorithm
 * **trjradialdistver.m** - Matlab function source file of the radial distance algorithm
### /miscellaneous
 * **shapecreate.m** - Matlab function source file of the custom shapefile creator
 * **trjstats.m** - Matlab function source file of the trajectory stats analyzer
 * **trjterrfilt.m** - Matlab function source file of the terrain-trajectory separation analyzer and filter
 * **trj2csv_ugcs.m** - Matlab function source file of the trajectory csv file creator
 * **demresol.m** - Matlab function source file of the digital elevation model resolution calculator
 * **trjascentlim.m** - Matlab function source file of the UAV ascent limit verifier

## data
Input and output data files structure for the planner and analyzer.

# QGIS shapefiles
For use of exported shapefiles from QGIS only the *.shp, *.shx and *.dbf files are necessary.

# Author
Tomáš Horeličan - horelican.t@gmail.com
