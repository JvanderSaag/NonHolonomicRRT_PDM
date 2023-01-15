from bin.ObstacleCreator_class import ObstacleCreator
from bin.Scenario_class import Scenario
from shapely.geometry import Point
from bin.RRT import RRT

# Define scenario   
TestScenario = Scenario("ParallelParking_Scenario", env_width=20, env_height=40, boundary_collision=True)

# Set vehicle parameters
TestScenario.set_vehicle(1/4.39, 4.5, 2)  # Max curvature, length, width

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
TestScenario.set_obstacles(obstacles)

# Define start and goal
start, start_yaw = (5, 5), 90
goal, goal_yaw = (13, 28.25), 90
TestScenario.set_start_goal(start, start_yaw, goal, goal_yaw)

Run = False
if Run:

    # Perform RRT
    RRT(2000, TestScenario, force_return_tree=True, step_size=10)

    # Plot scenario
    TestScenario.plot_scenario(plot_all_trees=True)

    # # Save scenario
    TestScenario.write_csv("ReedsSchepp")


# Save scenario
#TestScenario.read_csv("ReedsSchepp_43.53_2k")
TestScenario.plot_scenario()
