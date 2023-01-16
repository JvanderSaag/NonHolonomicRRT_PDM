# NonHolonomicRRT_PDM
Implementation of a complete Trajectory planning & tracking pipeline for a non-holonomic ground vehicle which utilises a path planner based on RRT* algorithm and an MPC controller. Created for the course Planning &amp; Decision Making (RO47005) at the Technische Universiteit Delft.
This project uses a modified version of RRT* with Reeds Shepp and Dubins connectors.


# Description
The project's main objective is to demonstrate an RRT* motion planner for a non-holonomic system, and to compare the difference in performance when using different types of connector functions. Furthermore a linear MPC controller is implemented in order to track the planned trajectory and visualise a vehicle following the calculated path.

## Files
The project is structured in multiple folders and files. 

### Scenarios
Multiple scenarios where prebuilt to give an example on how to create custom scenarios as well as serve for proof of concept and evalutation.
The results of these scenarios can be saved and loaded with the help of csv files saved in the Saved_scenario folder.

### Bin Folder 
The bin folder containts the main content of the project. The files here are the back bone of the RRT and simulation process. The files found in this folder are:

- RRT.py: RRT and RRT* implemention used as a global planner to find trajectories.
- Scenario_class.py: Defines a scenario class in order to set the environment parameters.
- ObstacleCreator_class.py: Class defined to allow ease of use of shapely by user without the need to know how to use the package.
- Reeds_Shepp_Curves.py: Functions used to connect a new point in RRT using Reeds Shepp curves.
- dublins_path_planner.py: Functions used to connect a new point in RRT using Dublins curves
- Controller.py: Functions to implement the state-space kinematic model and the linear MPC controller using cvxpy
- run_controller.py: Contains the main simulation loop for trajectory tracking
- draw.py: Helper functions to draw the 2D vehicle in simulation
- csv_utils.py: A dict linking scenarios to appropriate csv files

### Legacy folder
This folder contains old version of certain part of the code and aren't supported anymore.

### Plots
This folder contains some of the saved plots and videos for the tested scenarios.


# Dependencies
The following packages are needed to run this project:
- matplotlib  
- numpy  
- pandas 
- pip
- python
- wheel 
- scipy
- pygame
- cvxpy
- cvxopt
- shapely
- tqdm

This project was made using the latest version of these packages. The correct functioning of this program is not guaranteed with earlier versions of these packages.

# Installation
The program was tested to be fully functional on Linux (Ubuntu). We recommend using a virtual environment to execute these scripts. The repository contains a conda environment.yml file. The environment can be created as follows.

- Download and install Anaconda (if you do not use conda, you can also just make a python virtual environment with the packages mentioned in dependecies)
- Open Anaconda Prompt (Anaconda3) or your preferred terminal program
- In the terminal use cd change to the directory where you extracted the folder and where the environment.yml file exits
- Follow these commands

```
$ conda env create -f environment.yml

$ python -m ipykernel install --user --name=python3

$ conda activate group13_pdm_project
```
- Execute any of the provided test scenarios or follow the instruction mentioned in the next section. 
- Execute main.py to run the MPC controller on any of the saved paths in the `Saved_scenarios` folder. This script can be modified to execute any of the saved scenarios. Just change the name of the scenario in the main.py to the one you wish to run.


# Usage
In order to understand how to run this project, scenarios were provided pre-built and only require to be ran to work. The xxx_scenario.py files set up a simple environment and run RRT or RRT* as per the options specified to the function.

For creating a new scenario file the following imports are required:

```
from bin.ObstacleCreator_class import ObstacleCreator
from bin.Scenario_class import Scenario
from shapely.geometry import Point
from bin.RRT import RRT
```
The following is a small guide on how the various functions can be used to build a scenario

## Obstacle creation
Calling `ObstacleCreator()` initialises the class for the obstacles.

Obstacles can be created using 2 functions:

- `ObstacleCreator.create_rectangle(width, height, (x, y))`: Creates a rectangle using it s width and height and the bottom left corner cordinates.
- `ObstacleCreator.create_polygon([(x1,y1), (x2,y2)])`: Creates a polygon by giving a list of coordinates

Once the obstacles where created using the above function they need to be return in a list format using the function:

`ObstacleCreator.return_obstacles()`

## Scenario creation
First the scenario class must be initialised with:

`simple_Scenario = Scenario("Example_name", env_width=20, env_height=20, boundary_collision=True)`

When initialising, arguments must be provided for the name, width, and height of the environement. If it is desired that the boundary of the environement be considered when checking collision witht the vehicule, set `boundary_collision` to True. It is set to False by default.

Multiple functions must then be used to set-up the scenario:
- `simple_Scenario.set_start_goal(start, start_yaw, goal, goal_yaw)`: This sets up the start and goal positions. The yaw values must be in degrees between 0 and 360. The start and goal arguments must be a (x, y) tuple.
- `simple_Scenario.set_obstacles(obstacles)`: This sets the obstacle by giving it the list of obstacle discussed previously. (setting up obstacles is optionnal)
- `simple_Scenario.set_vehicle(0.6, width=2, length=4)`: This sets up the vehicule in the scenario class. The first argument is the maximum curve radius of the vehicule. Following this the width and length of the vehicle can be set.
- `simple_Scenario.plot_scenario()`: This allows the plotting of the resulting path. If the entire tree is desired use the argument `plot_all_trees=True`

## RRT
Once the scenario was set up it is possible to run the RRT algorithm using the following algorithm:

`RRT(N_iter, simple_Scenario)`

Only 2 arguments must be passed to the RRT function for it to work. N_iter: the number of iteration to run. simple_Scenario: the scenario object created witht the function from the previous section.

Multiple options can be used to run varying versions of RRT. These are the arguments and what they do:
- step_size (float, default:float('inf')): This limits how far new points may be generated.
- distance_tolerance (float, default:1): This describes the range at which a point is considered close enough to the goal.
- star (bool, default:True): If True RRT* will be used, if False RRT will be used.
- non_holonomic (bool, default:True): if True non holonomic connectors will be used, else euclidian connectors will be used.
- force_return_tree (bool, default:False): If set True the entire tree will be return to the scenario object. (Must be set to True to force the plotting of the tree in the scenario clas function)
- backwards (bool, default:True): If set to True Reeds Shepp connectors will be used, if set to false Dublin connectors will be used.


## Saving and reading saved paths
In order to save a scenario add the following line after running RRT:

`simple_Scenario.write_csv('example_name')` 

After a path has been saved, it can be read and plotted without running the RRT function. If a saved path 
is to be loaded, remove the RRT() and write_csv() functions from the scenario file, and use:

 `simple_Scenario.read_csv('example_name')`
 

To return the coordinates of the path that are saved in the csv file, the following command can be used:

`simple_scenario.return_path_coords()`

This returns a list of tuples, where each tuple represents (x, y, yaw, reversing). x and y indicate the coordinates of the point, where (x, y) = (0, 0) is the bottom left corner. The yaw is positive counter-clockwise on the domain [0, 360] degrees. The boolean 'reversing' indicates whether the car is moving backwards at that point. This is only relevant when using Reeds-Schepp path connectors.


# Example results

Planned paths:

![ParallelParking_Scenario_ReedShepp](https://user-images.githubusercontent.com/65740720/212574608-2ccd2a64-06f0-4685-9c92-5a33c0b463b4.png)


![Parking_Scenario_ReedsShepp](https://user-images.githubusercontent.com/65740720/212574616-371f7b1e-991c-42d9-a8f0-436e97891c2f.png)

Simulated trajectory:

https://user-images.githubusercontent.com/65740720/212574578-c381cac7-e9ac-45f4-9c58-c2c701d505f2.mp4




# References

Reeds Shepp curves planning implemented using existing code authored by Atsushi Sakai. The code was modified in order to output paths compatible with our implementation of RRT, and also features were added in order to increase the flexibility of the code. (https://github.com/AtsushiSakai/PythonRobotics/blob/master/PathPlanning/ReedsSheppPath/reeds_shepp_path_planning.py)

Dublins curves planning implemented using existing code authored by Huiming Zhou. The code was modified in order to output paths compatible with our implementation of RRT, and also features were added in order to increase the flexibility of the code.
(https://github.com/zhm-real/CurvesGenerator/blob/master/dubins_path.py)

MPC and draw functions implemented using existing code authored by Huming Zhou. The code was modified in order to use paths generated by the global planners from our implementation of RRT. (https://github.com/zhm-real/MotionPlanning)

## Future work
- Expanding RRT further in order to provide more options aswell as optimize it's efficiency. 
- Modifying the vehicles collistion calculation to be more realistic. When checking collision during RRT a circle is drawn around the vehicle. This is a known limitation that makes it impossible to have cars close by even when no collision is happening. 
