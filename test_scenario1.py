from bin.ObstacleCreator_class import ObstacleCreator
from bin.Scenario_class import Scenario
from bin.RRT import RRT

# Create obstacles
ObstacleCreator = ObstacleCreator()

ObstacleCreator.create_rectangle(10, 10, (5, 5))

obstacles = ObstacleCreator.return_obstacles()
start, start_yaw, goal, goal_yaw = (1, 1), 0, (19, 19), 0

simple_Scenario = Scenario("Test1", env_width=20, env_height=20, boundary_collision=True)
simple_Scenario.set_obstacles(obstacles)
simple_Scenario.set_start_goal(start, start_yaw, goal, goal_yaw)

RRT(1000, simple_Scenario, star=False, backwards=True)

#simple_Scenario.write_csv('test1')
#simple_Scenario.read_csv('test1', set_path=True)
simple_Scenario.plot_scenario()