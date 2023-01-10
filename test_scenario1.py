import ObstacleCreator_class, Scenario_class
from shapely.geometry import Point
from RRT import RRT

# Create obstacles
ObstacleCreator = ObstacleCreator_class.ObstacleCreator()

ObstacleCreator.create_rectangle(10, 10, (5, 5))

obstacles = ObstacleCreator.return_obstacles()
start, goal = Point(1, 1), Point (19, 19)

simple_Scenario = Scenario_class.Scenario(env_width=20, env_height=20, boundary_collision=True)
simple_Scenario.set_obstacles(obstacles)
<<<<<<< HEAD
simple_Scenario.set_start_goal(start, goal)
RRT(1000, simple_Scenario, star=False, non_holonomic=False)
simple_Scenario.plot_scenario(plot_all_trees=True)
=======
simple_Scenario.set_start_goal(start, start_yaw, goal, goal_yaw)

RRT(1000, simple_Scenario)
simple_Scenario.plot_scenario()
>>>>>>> ec0d08a46038fe6fb10b52f9c53a3ba7d3e9be35
