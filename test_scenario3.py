from bin.ObstacleCreator_class import ObstacleCreator
from bin.Scenario_class import Scenario
from shapely.geometry import Point
from bin.RRT import RRT

# Create obstacles
ObstacleCreator = ObstacleCreator()

ObstacleCreator.create_polygon([(5, 0), (5, 12.5), (8.5, 12.5), (8.5, 5), (15, 5), (15, 0)])
ObstacleCreator.create_polygon([(5, 20), (5, 17.5), (11.5, 17.5), (11.5, 10), (15, 10), (15, 20)])

obstacles = ObstacleCreator.return_obstacles()
start, start_yaw, goal, goal_yaw = Point(1, 1), 0, Point (19, 19), 0

simple_Scenario = Scenario(env_width=20, env_height=20, boundary_collision=True)
simple_Scenario.set_obstacles(obstacles)
simple_Scenario.set_start_goal(start, start_yaw, goal, goal_yaw)
simple_Scenario.set_vehicle(0.6)

RRT(1000, simple_Scenario)
simple_Scenario.plot_scenario()