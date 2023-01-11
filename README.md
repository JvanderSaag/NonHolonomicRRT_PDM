# NonHolonomicRRT_PDM
Implementation of a RRT algorithm for a non-holonomic parking robot, created for the course Planning &amp; Decision Making (RO47005) at the Technische Universiteit Delft.


return_coordinates of the scenario class returns the coordinates of all obstacles (a list of list, each nested list is another obstacle), the coordinates of the path (with linear segments connecting each point), the coordinate of the start, and the coordinate of the goal. All coordinates are given as (x, y), with the origin being at the bottom left of the environment.

To access the gym env: 

Clone gym_envs_urdf package: https://github.com/maxspahn/gym_envs_urdf
Install poetry: https://python-poetry.org/docs/

Run poetry via: $poetry shell

# Description

## Files
The project is structured in multiple folders and files. 

### Bin Folder 
The bin folder containts the main content of the project. The files here are the back bone of the RRT and simulation process. The files found in this folder are:

- RRT.py: RRT and RRT* implemention used as a global planner to find trajectories.
- Scenario_class.py: defines a scenario class in order to set the environment parameters.
- ObstacleCreator_class.py: class defined to allow ease of use of shapely by user without the need to know how to use the package.

# Dependencies
The following packages are needed to run this project:

- math
- [shapely](https://shapely.readthedocs.io/en/stable/manual.html)
- [matplotlib](https://pypi.org/project/matplotlib/)
- [numpy](https://numpy.org/)
- [tqdm](https://github.com/tqdm/tqdm)

This project was made using the latest version of these packages. The correct functionment of this program is not garantied with earlier versions of these packages.

# Usage

# References

Reeds Shepp curves planning implemented using existing code authored by Atsushi Sakai. The code was modified in order to output paths compatible with our implementation of RRT, and also features were added in order to increase the flexibility of the code. (https://github.com/AtsushiSakai/PythonRobotics/blob/master/PathPlanning/ReedsSheppPath/reeds_shepp_path_planning.py)

## Future work