import gym
from urdfenvs.robots.prius import Prius
from urdfenvs.sensors.obstacle_sensor import ObstacleSensor
from gym_envs_urdf.examples.scene_objects.obstacles import (
    sphereObst1,
    urdfObst1,
    dynamicSphereObst3,
)
import numpy as np


def run_prius(n_steps=1000, render=False, goal=True, obstacles=True):
    robots = [
        Prius(mode="vel"),
    ]
    env = gym.make(
        "urdf-env-v0",
        dt=0.01, robots=robots, render=render
    )
    action = np.array([1.1, 10.1])
    pos0 = np.array([-1.0, 0.2, -1.0])
    ob = env.reset(pos=pos0)
    env.add_walls()
    env.add_shapes("GEOM_BOX", dim = [0.7, 1.0, 0.5], mass = 1e8,poses_2d = [[1,1,0]])
    env.add_shapes("GEOM_BOX", dim = [0.7, 1.0, 0.5], mass = 1e8,poses_2d = [[2,-2,0]])
    #env.add_obstacle(sphereObst1)
    #env.add_obstacle(urdfObst1)
    #env.add_obstacle(dynamicSphereObst3)
    print(f"Initial observation : {ob}")
    history = []
    for i in range(n_steps):
        ob, _, _, _ = env.step(action)
        if ob['robot_0']['joint_state']['steering'] > 0.3:
            action[1] = 0
        history.append(ob)
    env.close()
    return history

if __name__ == "__main__":
    run_prius(render=True)
