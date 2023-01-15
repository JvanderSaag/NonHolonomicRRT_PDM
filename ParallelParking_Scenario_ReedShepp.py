from bin.ObstacleCreator_class import ObstacleCreator
from bin.Scenario_class import Scenario
from shapely.geometry import Point
from bin.RRT import RRT

# Define scenario   
simple_Scenario = Scenario("ParallelParking_Scenario", env_width=20, env_height=40, boundary_collision=True)

# Set vehicle parameters
simple_Scenario.set_vehicle(1/4.39, 4.5, 2)  # Max curvature, length, width

# Create obstacles
ObstacleCreator = ObstacleCreator()
#obstacleCreator.create_rectangle(5, 40, (0, 0))
ObstacleCreator.create_rectangle(10, 40, (16.5, 0))
ObstacleCreator.create_rectangle(2, 4.5, (12, 1))
ObstacleCreator.create_rectangle(2, 4.5, (12, 7))
ObstacleCreator.create_rectangle(2, 4.5, (12, 18.5))
ObstacleCreator.create_rectangle(2, 4.5, (12, 33.5))
obstacles = ObstacleCreator.return_obstacles()

# Set obstacles in scenario
simple_Scenario.set_obstacles(obstacles)

# Define start and goal
start, start_yaw = (5, 5), 90
goal, goal_yaw = (13, 28.25), 90
simple_Scenario.set_start_goal(start, start_yaw, goal, goal_yaw)

Run = False
if Run:

    # Perform RRT
    RRT(2000, simple_Scenario, force_return_tree=True, step_size=10)

    # Plot scenario
    simple_Scenario.plot_scenario(plot_all_trees=True)

    # # Save scenario
    simple_Scenario.write_csv("ReedsSchepp")


# Save scenario
#Scenario1.read_csv("ReedsSchepp_19.29_1k")
simple_Scenario.plot_scenario()
