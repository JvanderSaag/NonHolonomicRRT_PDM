from bin.ObstacleCreator_class import ObstacleCreator
from bin.Scenario_class import Scenario
from bin.RRT import RRT

# Create obstacles
ObstacleCreator = ObstacleCreator()

ObstacleCreator.create_polygon([(3, 4), (5, 10), (10, 9)])
ObstacleCreator.create_polygon([(5, 12), (6, 16), (17, 19)])
ObstacleCreator.create_polygon([(12, 3), (14, 8), (18, 2)])
ObstacleCreator.create_polygon([(12, 11), (16, 10), (18, 15)])

obstacles = ObstacleCreator.return_obstacles()
start, start_yaw, goal, goal_yaw = (1, 1), 0, (19, 19), 0

simple_Scenario = Scenario("Test4", env_width=20, env_height=20, boundary_collision=True)
simple_Scenario.set_obstacles(obstacles)
simple_Scenario.set_start_goal(start, start_yaw, goal, goal_yaw)
simple_Scenario.set_vehicle(0.6)

RRT(500, simple_Scenario)
print(simple_Scenario.return_path_coords())
simple_Scenario.plot_scenario()
