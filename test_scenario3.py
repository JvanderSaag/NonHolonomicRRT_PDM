import ObstacleCreator_class, Scenario_class
from shapely.geometry import Point
from RRT import RRT

# Create obstacles
ObstacleCreator = ObstacleCreator_class.ObstacleCreator()

ObstacleCreator.create_polygon([(5, 0), (5, 12.5), (8.5, 12.5), (8.5, 5), (15, 5), (15, 0)])
ObstacleCreator.create_polygon([(5, 20), (5, 17.5), (11.5, 17.5), (11.5, 10), (15, 10), (15, 20)])

obstacles = ObstacleCreator.return_obstacles()
start, goal = Point(1, 1), Point (19, 19)

simple_Scenario = Scenario_class.Scenario(env_width=20, env_height=20, boundary_collision=True)
simple_Scenario.set_obstacles(obstacles)
simple_Scenario.set_start_goal(start, goal)

RRT(100, simple_Scenario)
simple_Scenario.plot_scenario()