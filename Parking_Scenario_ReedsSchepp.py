from bin.ObstacleCreator_class import ObstacleCreator
from bin.Scenario_class import Scenario
from bin.RRT import RRT

# Create obstacles
ObstacleCreator = ObstacleCreator()
y_pos = [0.5, 44.5]
y_pos_bis = [20, 25]
for y in y_pos:
    for i in range(14):
        if y == 0.5 and i in [2,4,5,8,9,12]:
            continue
        if y == 44.5 and i in [0,1,2,5,6,8,11]:
            continue
        ObstacleCreator.create_rectangle(2, 4.5, (1.5 + i * 3, y))
for y in y_pos_bis:
    for i in range(7):
        if y == 25 and i in [1,2,5]:
            continue
        if y == 20 and i in [3,6]:
            continue
        ObstacleCreator.create_rectangle(2, 4.5, (10.5 + i * 3, y))
for i in range(12):
    if i in [2,5,6,8,9]:
        continue
    ObstacleCreator.create_rectangle(4.5, 2, (44.5, 7 + i * 3))


obstacles = ObstacleCreator.return_obstacles()
start, start_yaw, goal, goal_yaw = (16, 2.75), 90, (19, 46.75), 90

simple_Scenario = Scenario("Parking_Scenario", env_width=50, env_height=50, boundary_collision=False)
simple_Scenario.set_obstacles(obstacles)
simple_Scenario.set_start_goal(start, start_yaw, goal, goal_yaw)
simple_Scenario.set_vehicle(1/4.39, 4.5, 2)

RRT(20000, simple_Scenario, force_return_tree=True)


#simple_Scenario.read_csv('ReddsShepp_star_11.15_2k', set_path=True)
simple_Scenario.plot_scenario(plot_all_trees=True)
simple_Scenario.write_csv('Dubins_star')