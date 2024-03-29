import numpy as np
import matplotlib.patches
import matplotlib.pyplot as plt
import math
from shapely import affinity
from bin import Controller
from bin import draw
import time as tt
from shapely.geometry import MultiLineString
from bin.csv_utils import csv_keys



def run_sim(simple_Scenario, name, animate=True):
    cx, cy, cyaw, reversing = simple_Scenario.read_csv(csv_keys[name], set_path=True)
    cyaw = np.deg2rad([-(360-i) if i>180 else i for i in cyaw])
    #cyaw = np.deg2rad(cyaw)
    sp = Controller.calc_speed_profile(cx, Controller.P.target_speed, reversing)
    ref_path = Controller.PATH(cx, cy, cyaw)
    node = Controller.Node(x=cx[0], y=cy[0], yaw=cyaw[0], v=0.0)

    time2 = 0.0
    x = [node.x]
    y = [node.y]
    yaw = [node.yaw]
    v = [node.v]
    t = [0.0]
    d = [0.0]
    a = [0.0]

    delta_opt, a_opt = None, None
    a_exc, delta_exc = 0.0, 0.0
    start_time = tt.time()
    if animate:
        manager=plt.get_current_fig_manager()
        manager.full_screen_toggle()
    while time2 < Controller.P.time_max:
        z_ref, target_ind = Controller.calc_ref_trajectory_in_T_step(node, ref_path, sp)

        z0 = [node.x, node.y, node.v, node.yaw]

        a_opt, delta_opt, x_opt, y_opt, yaw_opt, v_opt = Controller.linear_mpc_control(z_ref, z0, a_opt, delta_opt)

        if delta_opt is not None:
            delta_exc, a_exc = delta_opt[0], a_opt[0]

        node.update(a_exc, delta_exc, 1.0)
        time2 += Controller.P.dt

        x.append(node.x)
        y.append(node.y)
        yaw.append(node.yaw)
        v.append(node.v)
        t.append(time2)
        d.append(delta_exc)
        a.append(a_exc)

        dist = math.hypot(node.x - cx[-1], node.y - cy[-1])

        if dist < Controller.P.dist_stop and \
                abs(node.v) < Controller.P.speed_stop:
            break

        dy = (node.yaw - yaw[-2]) / (node.v * Controller.P.dt)
        steer = Controller.pi_2_pi(-math.atan(Controller.P.WB * dy))
        
        if animate:
            plt.cla()

            draw.draw_car(node.x, node.y, node.yaw, steer, Controller.P)
            for obstacle in simple_Scenario.obstacles:
                obstacle = affinity.translate(obstacle, yoff = (simple_Scenario.vehicle_length/2 - Controller.P.RB))
                plt.gca().add_patch(matplotlib.patches.Polygon(obstacle.exterior.coords, color="grey"))

            if simple_Scenario.start is not None and simple_Scenario.goal is not None:
                if simple_Scenario.vehicle_length != 0 and simple_Scenario.vehicle_width != 0: # If the vehicle size has been set, draw start and goal as vehicle
                #  plt.gca().add_patch(matplotlib.patches.Rectangle((simple_Scenario.start[0].x - Controller.P.RB, simple_Scenario.start[0].y - simple_Scenario.vehicle_width / 2),
                #                                              simple_Scenario.vehicle_length, simple_Scenario.vehicle_width, simple_Scenario.start[1], color='red', alpha=0.8, label='Start', rotation_point='center'))
                    plt.gca().add_patch(matplotlib.patches.Rectangle((simple_Scenario.start[0].x + simple_Scenario.vehicle_width / 2, simple_Scenario.start[0].y - Controller.P.RB),
                                                                simple_Scenario.vehicle_length, simple_Scenario.vehicle_width, simple_Scenario.start[1], color='red', alpha=0.8, label='Start', rotation_point='xy'))
                    plt.gca().add_patch(matplotlib.patches.Rectangle((simple_Scenario.goal[0].x + simple_Scenario.vehicle_width / 2, simple_Scenario.goal[0].y - Controller.P.RB), 
                                                                simple_Scenario.vehicle_length, simple_Scenario.vehicle_width, simple_Scenario.goal[1], color='green', alpha=0.8, label='Goal', rotation_point='xy'))
                else: # Draw start and goal as points
                    plt.scatter(simple_Scenario.start[0].x, simple_Scenario.start[0].y, s=50, c='g', marker='o', label='Start')
                    plt.scatter(simple_Scenario.goal[0].x, simple_Scenario.goal[0].y, s=60, c='r', marker='*', label='Goal')
            plt.gcf().canvas.mpl_connect('key_release_event',
                                        lambda event:
                                        [exit(0) if event.key == 'escape' else None])

            if x_opt is not None:
                plt.plot(x_opt, y_opt, color='darkviolet', marker='*')

            plt.plot(cx, cy, color='gray', label='Planned Path')
            plt.plot(x, y, '-b', label='Simulated Path')
            plt.plot(cx[target_ind], cy[target_ind])

            plt.legend()
            plt.axis("equal")
            plt.title(name)
            plt.pause(0.001)
            
    end_time = tt.time()
    print(f"Simulation Time: {-(start_time - end_time)}")

    path = MultiLineString(simple_Scenario.path)
    print(f"Length of Planned Trajectory: {path.length}")

    simulated_path_points = list(zip(x,y))
    simulated_path = MultiLineString([[simulated_path_points[i], simulated_path_points[i+1]] for i in range(len(simulated_path_points) - 1)])
    print(f"Length of Simulated Trajectory: {simulated_path.length}")
    
    if not animate:
        px = 1/plt.rcParams['figure.dpi']
        fig, ax = plt.subplots(figsize=(900*px, 900*px))

        # Set size of plot with correct aspect
        ax.set_aspect(aspect=1)
        
        # Set boundaries for drawing scenario
        offset =  simple_Scenario.vehicle_length/2 - Controller.P.RB
        plt.xlim([0, simple_Scenario.width])
        plt.ylim([0 + offset, simple_Scenario.height + offset])

        for obstacle in simple_Scenario.obstacles:
                obstacle = affinity.translate(obstacle, yoff = (simple_Scenario.vehicle_length/2 - Controller.P.RB))
                plt.gca().add_patch(matplotlib.patches.Polygon(obstacle.exterior.coords, color="grey"))
        
        # Draw start and goal
        if simple_Scenario.start is not None and simple_Scenario.goal is not None:
                if simple_Scenario.vehicle_length != 0 and simple_Scenario.vehicle_width != 0: # If the vehicle size has been set, draw start and goal as vehicle
                #  plt.gca().add_patch(matplotlib.patches.Rectangle((simple_Scenario.start[0].x - Controller.P.RB, simple_Scenario.start[0].y - simple_Scenario.vehicle_width / 2),
                #                                              simple_Scenario.vehicle_length, simple_Scenario.vehicle_width, simple_Scenario.start[1], color='red', alpha=0.8, label='Start', rotation_point='center'))
                    plt.gca().add_patch(matplotlib.patches.Rectangle((simple_Scenario.start[0].x + simple_Scenario.vehicle_width / 2, simple_Scenario.start[0].y - Controller.P.RB),
                                                                simple_Scenario.vehicle_length, simple_Scenario.vehicle_width, simple_Scenario.start[1], color='red', alpha=0.8, label='Start', rotation_point='xy'))
                    plt.gca().add_patch(matplotlib.patches.Rectangle((simple_Scenario.goal[0].x + simple_Scenario.vehicle_width / 2, simple_Scenario.goal[0].y - Controller.P.RB), 
                                                                simple_Scenario.vehicle_length, simple_Scenario.vehicle_width, simple_Scenario.goal[1], color='green', alpha=0.8, label='Goal', rotation_point='xy'))
                else: # Draw start and goal as points
                    plt.scatter(simple_Scenario.start[0].x, simple_Scenario.start[0].y, s=50, c='g', marker='o', label='Start')
                    plt.scatter(simple_Scenario.goal[0].x, simple_Scenario.goal[0].y, s=60, c='r', marker='*', label='Goal')

        # Draw path
        plt.plot(cx, cy, color='gray', label='Planned Path')
        plt.plot(x, y, '-b', label='Simulated Path')
        plt.title(name)     
        plt.legend()
        plt.show()

    

    #plt.show()