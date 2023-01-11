"""
Dubins Path
"""

import math
import numpy as np
from scipy.spatial.transform import Rotation as Rot
from shapely.geometry import LineString



# class for PATH element
class PATH:
    def __init__(self, L, mode, x, y, yaw):
        self.L = L  # total path length [float]
        self.mode = mode  # type of each part of the path [string]
        self.x = x  # final x positions [m]
        self.y = y  # final y positions [m]
        self.yaw = yaw  # final yaw angles [rad]


# utils
def pi_2_pi(theta):
    while theta > math.pi:
        theta -= 2.0 * math.pi

    while theta < -math.pi:
        theta += 2.0 * math.pi

    return theta


def mod2pi(theta):
    return theta - 2.0 * math.pi * math.floor(theta / math.pi / 2.0)


def LSL(alpha, beta, dist):
    sin_a = math.sin(alpha)
    sin_b = math.sin(beta)
    cos_a = math.cos(alpha)
    cos_b = math.cos(beta)
    cos_a_b = math.cos(alpha - beta)

    p_lsl = 2 + dist ** 2 - 2 * cos_a_b + 2 * dist * (sin_a - sin_b)

    if p_lsl < 0:
        return None, None, None, ["L", "S", "L"]
    else:
        p_lsl = math.sqrt(p_lsl)

    denominate = dist + sin_a - sin_b
    t_lsl = mod2pi(-alpha + math.atan2(cos_b - cos_a, denominate))
    q_lsl = mod2pi(beta - math.atan2(cos_b - cos_a, denominate))

    return t_lsl, p_lsl, q_lsl, ["L", "S", "L"]


def RSR(alpha, beta, dist):
    sin_a = math.sin(alpha)
    sin_b = math.sin(beta)
    cos_a = math.cos(alpha)
    cos_b = math.cos(beta)
    cos_a_b = math.cos(alpha - beta)

    p_rsr = 2 + dist ** 2 - 2 * cos_a_b + 2 * dist * (sin_b - sin_a)

    if p_rsr < 0:
        return None, None, None, ["R", "S", "R"]
    else:
        p_rsr = math.sqrt(p_rsr)

    denominate = dist - sin_a + sin_b
    t_rsr = mod2pi(alpha - math.atan2(cos_a - cos_b, denominate))
    q_rsr = mod2pi(-beta + math.atan2(cos_a - cos_b, denominate))

    return t_rsr, p_rsr, q_rsr, ["R", "S", "R"]


def LSR(alpha, beta, dist):
    sin_a = math.sin(alpha)
    sin_b = math.sin(beta)
    cos_a = math.cos(alpha)
    cos_b = math.cos(beta)
    cos_a_b = math.cos(alpha - beta)

    p_lsr = -2 + dist ** 2 + 2 * cos_a_b + 2 * dist * (sin_a + sin_b)

    if p_lsr < 0:
        return None, None, None, ["L", "S", "R"]
    else:
        p_lsr = math.sqrt(p_lsr)

    rec = math.atan2(-cos_a - cos_b, dist + sin_a + sin_b) - math.atan2(-2.0, p_lsr)
    t_lsr = mod2pi(-alpha + rec)
    q_lsr = mod2pi(-mod2pi(beta) + rec)

    return t_lsr, p_lsr, q_lsr, ["L", "S", "R"]


def RSL(alpha, beta, dist):
    sin_a = math.sin(alpha)
    sin_b = math.sin(beta)
    cos_a = math.cos(alpha)
    cos_b = math.cos(beta)
    cos_a_b = math.cos(alpha - beta)

    p_rsl = -2 + dist ** 2 + 2 * cos_a_b - 2 * dist * (sin_a + sin_b)

    if p_rsl < 0:
        return None, None, None, ["R", "S", "L"]
    else:
        p_rsl = math.sqrt(p_rsl)

    rec = math.atan2(cos_a + cos_b, dist - sin_a - sin_b) - math.atan2(2.0, p_rsl)
    t_rsl = mod2pi(alpha - rec)
    q_rsl = mod2pi(beta - rec)

    return t_rsl, p_rsl, q_rsl, ["R", "S", "L"]


def RLR(alpha, beta, dist):
    sin_a = math.sin(alpha)
    sin_b = math.sin(beta)
    cos_a = math.cos(alpha)
    cos_b = math.cos(beta)
    cos_a_b = math.cos(alpha - beta)

    rec = (6.0 - dist ** 2 + 2.0 * cos_a_b + 2.0 * dist * (sin_a - sin_b)) / 8.0

    if abs(rec) > 1.0:
        return None, None, None, ["R", "L", "R"]

    p_rlr = mod2pi(2 * math.pi - math.acos(rec))
    t_rlr = mod2pi(alpha - math.atan2(cos_a - cos_b, dist - sin_a + sin_b) + mod2pi(p_rlr / 2.0))
    q_rlr = mod2pi(alpha - beta - t_rlr + mod2pi(p_rlr))

    return t_rlr, p_rlr, q_rlr, ["R", "L", "R"]


def LRL(alpha, beta, dist):
    sin_a = math.sin(alpha)
    sin_b = math.sin(beta)
    cos_a = math.cos(alpha)
    cos_b = math.cos(beta)
    cos_a_b = math.cos(alpha - beta)

    rec = (6.0 - dist ** 2 + 2.0 * cos_a_b + 2.0 * dist * (sin_b - sin_a)) / 8.0

    if abs(rec) > 1.0:
        return None, None, None, ["L", "R", "L"]

    p_lrl = mod2pi(2 * math.pi - math.acos(rec))
    t_lrl = mod2pi(-alpha - math.atan2(cos_a - cos_b, dist + sin_a - sin_b) + p_lrl / 2.0)
    q_lrl = mod2pi(mod2pi(beta) - alpha - t_lrl + mod2pi(p_lrl))

    return t_lrl, p_lrl, q_lrl, ["L", "R", "L"]


def interpolate(ind, l, m, maxc, ox, oy, oyaw, px, py, pyaw, directions):
    if m == "S":
        px[ind] = ox + l / maxc * math.cos(oyaw)
        py[ind] = oy + l / maxc * math.sin(oyaw)
        pyaw[ind] = oyaw
    else:
        ldx = math.sin(l) / maxc
        if m == "L":
            ldy = (1.0 - math.cos(l)) / maxc
        elif m == "R":
            ldy = (1.0 - math.cos(l)) / (-maxc)

        gdx = math.cos(-oyaw) * ldx + math.sin(-oyaw) * ldy
        gdy = -math.sin(-oyaw) * ldx + math.cos(-oyaw) * ldy
        px[ind] = ox + gdx
        py[ind] = oy + gdy

    if m == "L":
        pyaw[ind] = oyaw + l
    elif m == "R":
        pyaw[ind] = oyaw - l

    if l > 0.0:
        directions[ind] = 1
    else:
        directions[ind] = -1

    return px, py, pyaw, directions


def generate_local_course(L, lengths, mode, maxc, step):
    point_num = int(L / step) + len(lengths) + 3

    px = [0.0 for _ in range(point_num)]
    py = [0.0 for _ in range(point_num)]
    pyaw = [0.0 for _ in range(point_num)]
    directions = [0 for _ in range(point_num)]
    ind = 1

    if lengths[0] > 0.0:
        directions[0] = 1
    else:
        directions[0] = -1

    if lengths[0] > 0.0:
        d = step
    else:
        d = -step

    ll = 0.0

    for m, l, i in zip(mode, lengths, range(len(mode))):
        if l > 0.0:
            d = step
        else:
            d = -step

        ox, oy, oyaw = px[ind], py[ind], pyaw[ind]

        ind -= 1
        if i >= 1 and (lengths[i - 1] * lengths[i]) > 0:
            pd = -d - ll
        else:
            pd = d - ll

        while abs(pd) <= abs(l):
            ind += 1
            px, py, pyaw, directions = \
                interpolate(ind, pd, m, maxc, ox, oy, oyaw, px, py, pyaw, directions)
            pd += d

        ll = l - pd - d  # calc remain length

        ind += 1
        px, py, pyaw, directions = \
            interpolate(ind, l, m, maxc, ox, oy, oyaw, px, py, pyaw, directions)

    if len(px) <= 1:
        return [], [], [], []

    # remove unused data
    while len(px) >= 1 and px[-1] == 0.0:
        px.pop()
        py.pop()
        pyaw.pop()
        directions.pop()

    return px, py, pyaw, directions


def planning_from_origin(gx, gy, gyaw, curv, step):
    D = math.hypot(gx, gy)
    d = D * curv

    theta = mod2pi(math.atan2(gy, gx))
    alpha = mod2pi(-theta)
    beta = mod2pi(gyaw - theta)

    planners = [LSL, RSR, LSR, RSL, RLR, LRL]

    path_list = []
    for planner in planners:
        t, p, q, mode = planner(alpha, beta, d)

        if t is None:
            continue

        cost = (abs(t) + abs(p) + abs(q))
        lengths = [t, p, q]
        x_list, y_list, yaw_list, directions = generate_local_course(
            sum(lengths), lengths, mode, curv, step)
        path_list.append([x_list, y_list, yaw_list, mode, cost])

    return path_list


def calc_dubins_path(sx, sy, syaw, gx, gy, gyaw, curv, step=0.1):
    goal_x = gx - sx
    goal_y = gy - sy

    l_rot = Rot.from_euler('z', syaw).as_matrix()[0:2, 0:2]
    le_xy = np.stack([goal_x, goal_y]).T @ l_rot
    le_yaw = gyaw - syaw

    possible_paths = []
    path_list = planning_from_origin(
        le_xy[0], le_xy[1], le_yaw, curv, step)
    for path in path_list:
        lp_x, lp_y, lp_yaw, mode, lengths = path
        rot = Rot.from_euler('z', -syaw).as_matrix()[0:2, 0:2]
        converted_xy = np.stack([lp_x, lp_y]).T @ rot
        x_list = converted_xy[:, 0] + sx
        y_list = converted_xy[:, 1] + sy
        yaw_list = [pi_2_pi(i_yaw + syaw) for i_yaw in lp_yaw]
        possible_paths.append(PATH(lengths, mode, x_list, y_list, yaw_list))
    return possible_paths

def plan_dubins_path(sx, sy, syaw, gx, gy, gyaw, maxc, step=0.2):
    paths = []
    for curve_rate in maxc:
        paths += calc_dubins_path(sx, sy, syaw, gx, gy, gyaw, curve_rate, step)
    if paths is None:
        return None # could not generate any path

    # search minimum cost path
    linestring_list = []
    for path in paths:
        x, y = np.array(path.x), np.array(path.y)
        coord = np.vstack((x, y)).T
        linestring_list.append(LineString(coord))
    return sorted(linestring_list, key=lambda x: x.length)
