from bin.ObstacleCreator_class import ObstacleCreator
from bin.Scenario_class import Scenario
from bin.RRT import RRT

# Create obstacles
ObstacleCreator = ObstacleCreator()

ObstacleCreator.create_rectangle(10.5, 22.5, (0,0))
ObstacleCreator.create_rectangle(10.5, 25, (0,37))
ObstacleCreator.create_rectangle(20, 22.5, (25,0))
ObstacleCreator.create_rectangle(20, 25, (25,37))


obstacles = ObstacleCreator.return_obstacles()
start, start_yaw, goal, goal_yaw = (18, 5), 90, (52.5, 45), 90

simple_Scenario = Scenario("Street_Scenario", env_width=60, env_height=50, boundary_collision=False)
simple_Scenario.set_obstacles(obstacles)
simple_Scenario.set_start_goal(start, start_yaw, goal, goal_yaw)
simple_Scenario.set_vehicle(1/4.39, width=2, length=4.5)

Run = False

if Run:
  RRT(4000, simple_Scenario, force_return_tree=True)
  simple_Scenario.plot_scenario(plot_all_trees=True)
  simple_Scenario.write_csv('ReedsShepp') 

#simple_Scenario.read_csv('ReedsShepp_11.13_4k')
   
