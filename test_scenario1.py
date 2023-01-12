from bin.ObstacleCreator_class import ObstacleCreator
from bin.Scenario_class import Scenario
from bin.RRT import RRT

# Create obstacles
ObstacleCreator = ObstacleCreator()

ObstacleCreator.create_rectangle(20, 20, (15, 15))

obstacles = ObstacleCreator.return_obstacles()
start, start_yaw, goal, goal_yaw = (5, 5), 0, (45, 45), 0

simple_Scenario = Scenario("Test1", env_width=50, env_height=50, boundary_collision=True)
simple_Scenario.set_obstacles(obstacles)
simple_Scenario.set_start_goal(start, start_yaw, goal, goal_yaw)
simple_Scenario.set_vehicle(1/4.39, 4.5, 2)

RRT(1000, simple_Scenario, backwards=True)

simple_Scenario.write_csv('test1')
#simple_Scenario.read_csv('test1', set_path=True)

simple_Scenario.plot_scenario()