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

TestScenario = Scenario("Street_Scenario", env_width=60, env_height=50, boundary_collision=False)
TestScenario.set_vehicle(1/4.39, width=2, length=4.5)
TestScenario.set_obstacles(obstacles)
TestScenario.set_start_goal(start, start_yaw, goal, goal_yaw)
  
Run = False
if Run:
    RRT(20000, TestScenario, star=True, backwards=False, force_return_tree=True, step_size=20)
    TestScenario.plot_scenario(plot_all_trees=True)
    TestScenario.write_csv('Dubins')    

#TestScenario.read_csv('Dubins_39.54_20k', set_path=True)
TestScenario.plot_scenario()