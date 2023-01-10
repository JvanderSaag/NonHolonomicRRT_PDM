import ObstacleCreator_class, Scenario_class
from shapely.geometry import Point
from RRT import RRT

# Create obstacles
ObstacleCreator = ObstacleCreator_class.ObstacleCreator()

ObstacleCreator.create_polygon([(3, 4), (5, 10), (10, 9)])
ObstacleCreator.create_polygon([(5, 12), (6, 16), (17, 19)])
ObstacleCreator.create_polygon([(12, 3), (14, 8), (18, 2)])
ObstacleCreator.create_polygon([(12, 11), (16, 10), (18, 15)])

obstacles = ObstacleCreator.return_obstacles()
start, start_yaw, goal, goal_yaw = Point(1, 1), 0, Point (19, 19), 0

simple_Scenario = Scenario_class.Scenario(env_width=20, env_height=20, boundary_collision=True)
simple_Scenario.set_obstacles(obstacles)
simple_Scenario.set_start_goal(start, start_yaw, goal, goal_yaw)
simple_Scenario.set_vehicle(0.6)

RRT(1000, simple_Scenario)
simple_Scenario.plot_scenario()