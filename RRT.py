""" These are the functions used for the RRT algorithm, running the main RRT() function will
    update the scenario class with the final path. Step size and distance tolerance can both be changed when calling RRT().
"""

import numpy as np
from shapely.geometry import Point, LineString
from tqdm import tqdm

class TreeNode: # Tree Node class that RRT uses
    def __init__(self, point, yaw=None):
        self.point = point # This should be shapely PointObject, containing coordinate informatioj
        self.yaw = yaw # Orientation of vehicle
        self.parent = None # Children, list of nodes
        self.path_to_parent = None # Path from parent, list of paths
        self.children = [] # Children, only used to draw the entire RRT tree


def RRT(N_iter, scenario, step_size=float('inf'), dist_tolerance=0.5): # RRT using TreeNodes
    start_Node, goal_Node = TreeNode(scenario.start), TreeNode(scenario.goal)

    for n in tqdm(range(N_iter)): # Max N_iter iterations
        # Sample a random point in the space
        sampled_point = Point(rand_coords(scenario.width, scenario.height))
        sampled_Node = TreeNode(sampled_point)

        if not scenario.collision_free(sampled_point): # If the sampled point collides
            continue # Continue to next iteration

        # Find the closest node to sampled point
        parent_Node, path_to_parent, _ = find_closest_Node(scenario, start_Node, sampled_Node, step_size)
                        
        if parent_Node is not None:  # If a nearest node is found      
            if path_to_parent.length > step_size: # In the case that this parent node is not within the step size
                # Find a new point on the connecting line that is within the radius
                new_Node_inradius = TreeNode(path_to_parent.interpolate(step_size))
                new_connect_line = LineString([parent_Node.point, new_Node_inradius.point])
                # Update the sampled Node to this new point
                sampled_Node = new_Node_inradius
                path_to_parent = new_connect_line
                # min_length = new_connect_line.length NOT IN USE AT THE MOMENT    
              
            sampled_Node.parent = parent_Node # Update the sampled_Nodes parent, path
            sampled_Node.path_to_parent = path_to_parent   
            parent_Node.children.append(sampled_Node) # add children to the parent

            # If the point is close to the goal, save the path and total tree
            if sampled_Node.point.distance(goal_Node.point) < dist_tolerance:
                final_path = extract_path(sampled_Node)
                total_tree = extract_all_edges(start_Node)
                
                # Update class attributes
                scenario.set_path(final_path)
                scenario.set_totaltree(total_tree)

                print(f"\nRRT finished within {n} iterations")
                return

    
    # If it does not converge, this code will be reached
    raise Exception("\nRRT could not find a suitable path within the given number of iterations. Please try again.\nIf it consistently fails to complete, increase the number of iterations.")


def rand_coords(width, height): # Generate random coordinates
    x = np.random.randint(0,width*100,1) / 100
    y = np.random.randint(0,height*100,1) / 100
    return Point(x, y)


def find_closest_Node(scenario, start_Node, new_Node, min_length=float('inf')): # Find closest Node in Tree
    nearest_Node, shortest_path = None, None # Initalise nearest Node and shortest path as None

    # Recursively check all children
    if start_Node.children: # If Node has children
        for child in start_Node.children: # Iterate over the children nodes
            temp_Node, temp_path, temp_length = find_closest_Node(scenario, child, new_Node, min_length=min_length)
            if temp_length < min_length:
                nearest_Node, shortest_path, min_length = temp_Node, temp_path, temp_length
    
    # Either there are no children, or the recursive search is done (this code will be reached) 
    # CONNECTOR FUNCTION #
    connect_line = LineString([start_Node.point, new_Node.point]) # Create a line that connects to the new Node

    # COLLISION CHECK #
    if scenario.collision_free(connect_line): # If the line does not collide with the environment
        length = connect_line.length # Find length of connecting line

        # If this length is less than the saved min, and the limiting radius
        if length < min_length:
            nearest_Node = start_Node # set nearest node
            shortest_path = connect_line # define the shortest path
            min_length = length # then update minimum length
       
    return nearest_Node, shortest_path, min_length


def extract_all_edges(start_Node, total_tree=[]): # Extract all edges from tree
    if start_Node.children: # Iterate over all the children of the start_Node
        for child in start_Node.children: # For each child
            total_tree.append(child.path_to_parent) # Append path to total_tree
            extract_all_edges(child, total_tree) # Recursively repeat over all its children
    return total_tree
    

def extract_path(final_Node, final_path=[]): # Extract only final path from tree
    if final_Node.parent is not None: # As long as there is a parent
        final_path.append(final_Node.path_to_parent) # Append the path_to_parent to final path
        extract_path(final_Node.parent, final_path) # Recursively repeat one layer up
    return final_path

