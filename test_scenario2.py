from bin.ObstacleCreator_class import ObstacleCreator
from bin.Scenario_class import Scenario
from bin.RRT import RRT

# Create obstacles
ObstacleCreator = ObstacleCreator()

ObstacleCreator.create_rectangle(10, 7, (5, 0))
ObstacleCreator.create_rectangle(10, 7, (5, 13))

obstacles = ObstacleCreator.return_obstacles()
start, start_yaw, goal, goal_yaw = (1, 1), 0, (19, 19), 0

simple_Scenario = Scenario("Test2", env_width=20, env_height=20, boundary_collision=True)
simple_Scenario.set_obstacles(obstacles)
simple_Scenario.set_start_goal(start, start_yaw, goal, goal_yaw)
simple_Scenario.set_vehicle(0.6)

RRT(1000, simple_Scenario)
simple_Scenario.plot_scenario()
