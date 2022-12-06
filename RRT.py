import numpy as np
from shapely.geometry import Point, LineString
import matplotlib.pyplot as plt

def rand_coords(width, height):
    x = np.random.randint(0,width*100,1) / 100
    y = np.random.randint(0,height*100,1) / 100
    return Point(x, y)

# def is_collision_free(S, ver_1, ver_2):
#     path = shapely.geometry.LineString([ver_1, ver_2])
#     return S.collision_free(path)

def closest_point(scenario, tree, new_point, radius=float('inf')):
    dist_min, edge_min = float('inf'), None
    nearest_point, point_id = None, None

    for id, point in enumerate(tree.points):
        connect_line = LineString([point, new_point])
        if not scenario.collision_free(connect_line):
            continue

        dist = new_point.distance(point)
        if dist < dist_min and dist < radius:
            dist_min = dist
            nearest_point, point_id = point, id
            edge_min = connect_line
        
    return nearest_point, point_id, edge_min

def RRT(N_iter, scenario):
    search_tree = Tree(scenario.start, scenario.goal)

    for n in range(N_iter):
        sampled_point = Point(rand_coords(scenario.width, scenario.height))
        if not scenario.collision_free(sampled_point):
            continue
        
        nearest_point, nearest_point_id, edge = closest_point(scenario, search_tree, sampled_point)

        if nearest_point is not None:
            new_id = search_tree.add_point(sampled_point)
            dist = nearest_point.distance(sampled_point)
            search_tree.add_edge(nearest_point_id, new_id, dist, edge)    

    return search_tree.edges
class Tree:
    def __init__(self, start, goal):
        self.start = start
        self.goal = goal

        self.points = [start]
        self.edges = []

        self.neighbors = {0:[]}
        #self.point_id = {0:start}
    
    def add_point(self, new_point):
        if new_point not in self.points:
            id = len(self.points)
            self.points.append(new_point)
            #self.point_id[new_point] = id
            self.neighbors[id] = []
        return id

    def add_edge(self, prev_point_id, new_point_id, dist, edge_obj):
        self.edges.append(edge_obj)
        self.neighbors[prev_point_id].append((new_point_id, dist))
        self.neighbors[new_point_id].append((prev_point_id, dist))
