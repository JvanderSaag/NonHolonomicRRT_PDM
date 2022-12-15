import numpy as np
from shapely.geometry import Point, LineString

class TreeNode: # Tree Node class that RRT uses
    def __init__(self, point, yaw=None):
        self.point = point # This should be shapely PointObject, containing coordinate informatioj
        self.yaw = yaw # Orientation of vehicle
        self.parent = None # Children, list of nodes
        self.path_to_parent = None # Path from parent, list of paths
        self.children = [] # Children, only used to draw the entire RRT tree


def RRT(N_iter, scenario, dist_tolerance=0.5, plot_all_trees=False): # RRT using TreeNodes
    start_Node, goal_Node = TreeNode(scenario.start), TreeNode(scenario.goal)

    for n in range(N_iter):
        sampled_point = Point(rand_coords(scenario.width, scenario.height))
        sampled_Node = TreeNode(sampled_point)

        if not scenario.collision_free(sampled_point): # If the sampled point collides
            continue 
        
        parent_Node, path_to_parent, _ = find_closest_Node(scenario, start_Node, sampled_Node)

        if parent_Node is not None:            
            sampled_Node.parent = parent_Node
            sampled_Node.path_to_parent = path_to_parent   
            parent_Node.children.append(sampled_Node)

            # If the point is close to the goal, and the user does not want to plot all trees
            if sampled_Node.point.distance(goal_Node.point) < dist_tolerance and not plot_all_trees:
                final_path = extract_path(sampled_Node) # only final path will be plotted
                return final_path
    
    if plot_all_trees: # If user wants to plot all trees
        return extract_all_edges(start_Node)
 
    # If it does not converge, this code will be reached
    raise Exception("RRT could not find a suitable path within the given number of iterations")


def rand_coords(width, height): # Generate random coordinates
    x = np.random.randint(0,width*100,1) / 100
    y = np.random.randint(0,height*100,1) / 100
    return Point(x, y)


def find_closest_Node(scenario, start_Node, new_Node, radius=float('inf'), min_length=float('inf')): # Find closet Node in Tree
    nearest_Node, shortest_path = None, None

    # Recursively check all children
    if start_Node.children: # If Node has children
        for child in start_Node.children: # Iterate over the children nodes
            try:
                temp_Node, temp_path, temp_length = find_closest_Node(scenario, child, new_Node, min_length=min_length)
                if temp_length < min_length:
                    nearest_Node, shortest_path, min_length = temp_Node, temp_path, temp_length
            except StopIteration:
                continue
    
    # Either there are no children, or the recursive search is done (this code will be reached)
    # CONNECTOR FUNCTION
    connect_line = LineString([start_Node.point, new_Node.point]) # Create a line that connects to the new Node
    if scenario.collision_free(connect_line): # If the line does not collide with the environment
        length = connect_line.length # Find length of connecting line

        # If this length is less than the saved min, and the limiting radius
        if length < min_length and length < radius:  
            nearest_Node = start_Node # set nearest node
            shortest_path = connect_line # define the shortest path
            min_length = length # then update minimum length
    return nearest_Node, shortest_path, min_length


def extract_all_edges(start_Node, total_tree=[]): # Extract all edges from tree
    if start_Node.children:
        for child in start_Node.children:
            total_tree.append(child.path_to_parent)
            extract_all_edges(child, total_tree)
    return total_tree
    

def extract_path(final_Node, final_path=[]): # Extract only final path from tree
    if final_Node.parent is not None:
        final_path.append(final_Node.path_to_parent)
        extract_path(final_Node.parent, final_path)
    return final_path
