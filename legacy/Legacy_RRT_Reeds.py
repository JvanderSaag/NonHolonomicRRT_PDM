import numpy as np
from shapely.geometry import Point, LineString, MultiLineString
import matplotlib.pyplot as plt
import Reeds_Shepp_Curves


def rand_coords(width, height):
    x = np.random.randint(0,width*100,1) / 100
    y = np.random.randint(0,height*100,1) / 100
    return Point(x, y)

def closest_point(scenario, tree, new_point, new_point_orientation, radius=float('inf')):
    dist_min = float('inf')
    nearest_point, point_id, collection = None, None, None


    for id, point in enumerate(tree.points):
        maxc = 0.1
        sx, sy, syaw = new_point.x, new_point.y, new_point_orientation
        gx, gy, gyaw = point.x, point.y, tree.orientation[id]
        path_xs, path_ys, _, _, path_lenghts = Reeds_Shepp_Curves.reeds_shepp_path_planning(sx, sy, syaw, gx, gy, gyaw, maxc, step_size=0.2)   


        path_list = []
        for i in range(len(path_xs) - 1):
            path_list.append(LineString([Point(path_xs[i], path_ys[i]), Point(path_xs[i + 1], path_ys[i + 1])]))
        line_collection = MultiLineString(path_list)

        if not scenario.collision_free(line_collection):
            continue

        dist = sum(path_lenghts)
        if dist < dist_min:
            dist_min = dist
            nearest_point, point_id = point, id
            collection = line_collection
        
        
    return nearest_point, point_id, collection, dist_min

def RRT(N_iter, scenario):
    search_tree = Tree(scenario.start, scenario.goal)

    for n in range(N_iter):
        print(n)
        sampled_point = Point(rand_coords(scenario.width, scenario.height))
        sample_orientation = np.deg2rad(np.random.randint(-180,180,1))
        if not scenario.collision_free(sampled_point):
            continue
        
        nearest_point, nearest_point_id, collection, dist = closest_point(scenario, search_tree, sampled_point, sample_orientation)
        if nearest_point is not None:
            new_id = search_tree.add_point(sampled_point, sample_orientation)
            search_tree.add_paths(nearest_point_id, new_id, dist, collection)    

    return search_tree.paths
class Tree:
    def __init__(self, start, goal):
        self.start = start
        self.goal = goal

        self.points = [start]
        self.paths = []
        self.orientation = [0]

        self.neighbors = {0:[]}
        #self.point_id = {0:start}
    
    def add_point(self, new_point, orient):
        if new_point not in self.points:
            id = len(self.points)
            self.points.append(new_point)
            self.orientation.append(orient)
            #self.point_id[new_point] = id
            self.neighbors[id] = []
        return id

    def add_paths(self, prev_point_id, new_point_id, dist, paths):
        self.paths.append(paths)
        self.neighbors[prev_point_id].append((new_point_id, dist))
        self.neighbors[new_point_id].append((prev_point_id, dist))
