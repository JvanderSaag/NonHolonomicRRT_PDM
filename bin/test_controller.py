import os
import sys
import numpy as np
import matplotlib.patches
import matplotlib.pyplot as plt
import math
import Controller
import draw

sys.path.append('../NonHolonomicRRT_PDM')

from test_scenario1 import simple_Scenario

def main():
    cx, cy, cyaw = simple_Scenario.read_csv('test2', set_path=True)

    sp = Controller.calc_speed_profile(cx, cy, cyaw, Controller.P.target_speed)

    ref_path = Controller.PATH(cx, cy, cyaw)
    node = Controller.Node(x=cx[0], y=cy[0], yaw=cyaw[0], v=0.0)

    time = 0.0
    x = [node.x]
    y = [node.y]
    yaw = [node.yaw]
    v = [node.v]
    t = [0.0]
    d = [0.0]
    a = [0.0]

    delta_opt, a_opt = None, None
    a_exc, delta_exc = 0.0, 0.0

    while time < Controller.P.time_max:
        z_ref, target_ind = Controller.calc_ref_trajectory_in_T_step(node, ref_path, sp)

        z0 = [node.x, node.y, node.v, node.yaw]

        a_opt, delta_opt, x_opt, y_opt, yaw_opt, v_opt = Controller.linear_mpc_control(z_ref, z0, a_opt, delta_opt)

        if delta_opt is not None:
            delta_exc, a_exc = delta_opt[0], a_opt[0]

        node.update(a_exc, delta_exc, 1.0)
        time += Controller.P.dt

        x.append(node.x)
        y.append(node.y)
        yaw.append(node.yaw)
        v.append(node.v)
        t.append(time)
        d.append(delta_exc)
        a.append(a_exc)

        dist = math.hypot(node.x - cx[-1], node.y - cy[-1])

        if dist < Controller.P.dist_stop and \
                abs(node.v) < Controller.P.speed_stop:
            break

        dy = (node.yaw - yaw[-2]) / (node.v * Controller.P.dt)
        steer = Controller.pi_2_pi(-math.atan(Controller.P.WB * dy))
        
        plt.cla()
        draw.draw_car(node.x, node.y, node.yaw, steer, Controller.P)
        for obstacle in simple_Scenario.obstacles:
            plt.gca().add_patch(matplotlib.patches.Polygon(obstacle.exterior.coords, color="grey"))

        if simple_Scenario.start is not None and simple_Scenario.goal is not None:
            if simple_Scenario.vehicle_length != 0 and simple_Scenario.vehicle_width != 0: # If the vehicle size has been set, draw start and goal as vehicle
                plt.gca().add_patch(matplotlib.patches.Rectangle((simple_Scenario.start[0].x - Controller.P.RB, simple_Scenario.start[0].y - simple_Scenario.vehicle_width / 2),
                                                            simple_Scenario.vehicle_length, simple_Scenario.vehicle_width, simple_Scenario.start[1], color='red', alpha=0.8, label='Start', rotation_point='center'))
                plt.gca().add_patch(matplotlib.patches.Rectangle((simple_Scenario.goal[0].x - Controller.P.RB, simple_Scenario.goal[0].y - simple_Scenario.vehicle_width / 2), 
                                                            simple_Scenario.vehicle_length, simple_Scenario.vehicle_width, simple_Scenario.goal[1], color='green', alpha=0.8, label='Goal', rotation_point='center'))
            else: # Draw start and goal as points
                plt.scatter(simple_Scenario.start[0].x, simple_Scenario.start[0].y, s=50, c='g', marker='o', label='Start')
                plt.scatter(simple_Scenario.goal[0].x, simple_Scenario.goal[0].y, s=60, c='r', marker='*', label='Goal')
        plt.gcf().canvas.mpl_connect('key_release_event',
                                     lambda event:
                                     [exit(0) if event.key == 'escape' else None])

        if x_opt is not None:
            plt.plot(x_opt, y_opt, color='darkviolet', marker='*')

        plt.plot(cx, cy, color='gray')
        plt.plot(x, y, '-b')
        plt.plot(cx[target_ind], cy[target_ind])
        plt.axis("equal")
        #plt.title("Linear MPC, " + "v = " + str(round(node.v * 3.6, 2)))
        plt.pause(0.001)

    plt.show()
    

if __name__ == '__main__':
    main()
