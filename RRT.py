import numpy as np
import shapely.geometry
import matplotlib.pyplot as plt
from Scenario_class import Scenario

def rand_ver(area):
    width, height = area
    x = np.random.randint(0,width*100,1) / 100
    y = np.random.randint(0,height*100,1) / 100
    return x[0], y[0]

def is_collision_free(S, ver_1, ver_2):
    path = shapely.geometry.LineString([ver_1, ver_2])
    return S.collision_free(path)

def distance(ver_1, ver_2):
    return np.square((ver_1[0]-ver_2[0])**2 + (ver_1[1]-ver_2[1])**2)

def closest_ver(S, tree, ver, radius=float('inf')):
    dist_mem = float('inf')
    nearest_ver, ver_id = None, None

    for id, V in enumerate(tree.vertices):
        if not is_collision_free(S, ver, V):
            continue
        dist = distance(ver, V)
        if dist < dist_mem and dist < radius:
            dist_mem = dist
            nearest_ver, ver_id = V, id
    return nearest_ver, ver_id

def RRT(N_iter, start, goal, area, S):
    T = tree(start, goal)
    for n in range(N_iter):
        posx, posy = rand_ver(area)
        obj = shapely.geometry.Point(posx, posy)
        if not S.collision_free(obj):
            continue

        nearest_ver, nearest_ver_id = closest_ver(S, T, (posx, posy))

        if nearest_ver is not None:
            new_id = T.add_ver((posx, posy))
            dist = distance(nearest_ver, (posx, posy))
            T.add_edge(nearest_ver_id, new_id, dist)
            plt.plot(*shapely.geometry.LineString([nearest_ver, (posx, posy)]).xy)

class tree:
    def __init__(self, start, goal):
        self.start = start
        self.goal = goal

        self.vertices = [start]
        self.edges = []

        self.neighbors = {0:[]}
        self.ver_id = {start:0}
    
    def add_ver(self, new_ver):
        try:
            id = self.ver_id[new_ver]
        except:
            id = len(self.vertices)
            self.vertices.append(new_ver)
            self.ver_id[new_ver] = id
            self.neighbors[id] = []
        return id

    def add_edge(self, prev_ver, new_ver, dist):
        self.edges.append((prev_ver, new_ver))
        self.neighbors[prev_ver].append((new_ver, dist))
        self.neighbors[new_ver].append((prev_ver, dist))

    def plotting(self):
        pass


