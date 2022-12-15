import ObstacleCreator_class, Scenario_class
from shapely.geometry import Point
from RRT import RRT

# Create obstacles
ObstacleCreator = ObstacleCreator_class.ObstacleCreator()

ObstacleCreator.create_rectangle(10, 10, (5, 5))

obstacles = ObstacleCreator.return_obstacles()
start, goal = Point(1, 1), Point (19, 19)

simple_Scenario = Scenario_class.Scenario(env_width=20, env_height=20)
simple_Scenario.set_obstacles(obstacles)
simple_Scenario.set_start_goal(start, goal)
edges = RRT(500, simple_Scenario)
simple_Scenario.set_path(edges)
simple_Scenario.plot_scenario()