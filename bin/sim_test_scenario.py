from bin.ObstacleCreator_class import ObstacleCreator
from bin.Scenario_class import Scenario
from shapely.geometry import Point
from bin.RRT import RRT
from bin.Simulator import run_sim
import numpy as np

# Create obstacles
ObstacleCreator = ObstacleCreator()

ObstacleCreator.create_rectangle(10, 10, (5, 5))

obstacles = ObstacleCreator.return_obstacles()
start, start_yaw, goal, goal_yaw = Point(1, 1), 0, Point (19, 19), 0

simple_Scenario = Scenario(env_width=20, env_height=20, boundary_collision=True)
simple_Scenario.set_obstacles(obstacles)
simple_Scenario.set_start_goal(start, start_yaw, goal, goal_yaw)
simple_Scenario.set_vehicle(0.6, length=2, width=1)

# RRT(5000, simple_Scenario, star=False, non_holonomic=True)

# simple_Scenario.plot_scenario()
dummy_path = (np.vstack((15*np.sin(np.arange(0,2*np.pi,0.01)), 5*np.cos(np.arange(0,2*np.pi,0.01))))).T
# dummy_yaw = np.zeros(dummy_path.shape[0])
dummy_yaw = np.random.rand(dummy_path.shape[0],)

#Extracting path coords from csv
xs, ys, _ = simple_Scenario.read_csv('test1', set_path=True)
run_sim(simple_Scenario, dummy_path, dummy_yaw)