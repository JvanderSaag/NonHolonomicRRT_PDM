""" These are the functions used for the RRT* algorithm, running the main RRT() function will
    update the scenario class with the final path. Step size and distance tolerance can both be changed when calling RRT().
"""

import numpy as np
from shapely.geometry import Point, LineString
from tqdm import tqdm

class TreeNode: # Tree Node class that RRT uses
    def __init__(self, point, yaw=None):
        self.point = point # This should be shapely PointObject, containing coordinate informatioj
        self.yaw = yaw # Orientation of vehicle

        self.cost = 0 # Initial cost of zero
        self.parent = None # Children, list of nodes
        self.path_to_parent = None # Path from parent, list of paths
        self.children = [] # Children, only used to draw the entire RRT tree


def RRT(N_iter, scenario, step_size=float('inf'), dist_tolerance=0.2, goal_prob=0.05): # RRT using TreeNodes
    start_Node, goal_Node = TreeNode(scenario.start), TreeNode(scenario.goal)

    for n in tqdm(range(N_iter)): # Max N_iter iterations
        # Have a chance of picking the goal node
        if np.random.random_sample() < goal_prob:
            sampled_Node = goal_Node
        else:  # Otherwise, randomly sample a point in the environment
            sampled_Node = TreeNode(Point(rand_coords(scenario.width, scenario.height)))

         # If the sampled point collides with obstacles
        if not scenario.collision_free(sampled_Node.point):
            continue # Continue to next iteration

        # Find the closest node to sampled point
        parent_Node, path_to_parent, _ = find_best_Node(scenario, start_Node, sampled_Node, step_size)

        # Find nearby points x
        if parent_Node is not None:  # If a nearest node is found     
            if path_to_parent.length > step_size: # In the case that this parent node is not within the step size
                # Find a new point on the connecting line that is within the radius
                new_Node_inradius = TreeNode(path_to_parent.interpolate(step_size))
                new_connect_line = LineString([parent_Node.point, new_Node_inradius.point])

                # Update the sampled Node to this new point
                sampled_Node = new_Node_inradius
                path_to_parent = new_connect_line

            # Update the sampled_Node's parent, path and cost
            sampled_Node.parent = parent_Node 
            sampled_Node.path_to_parent = path_to_parent 
            sampled_Node.cost = parent_Node.cost + path_to_parent.length 

            # Add children to the parent
            parent_Node.children.append(sampled_Node)
            
    Nodes_near_goal = find_nearby_nodes(start_Node, goal_Node, dist_tolerance)
    
    shortest_path, min_cost = None, float('inf')
    
    # If it does not converge, this code will be reached
    if not Nodes_near_goal:
        raise Exception("\nRRT could not find a suitable path within the given number of iterations. Please try again.\nIf it consistently fails to complete, increase the number of iterations.")

    # Else a path has been found, find the shortest one if there are several
    for node in Nodes_near_goal:
        path = extract_path(node, path=[]) # Extract path to start node
        cost = 0
        for connector in path:
            cost += connector.length # Compute total cost of path
        if cost < min_cost: 
            shortest_path, min_cost = path, cost # Select only the shortest
    
    # Finally set final path and 'total' tree containing all edges
    final_path = shortest_path 
    total_tree = extract_all_edges(start_Node)

    # Update class attributes
    scenario.set_path(final_path)
    scenario.set_totaltree(total_tree)

    print(f"\nRRT finished within {n+1} iterations")
    return


def rand_coords(width, height): # Generate random coordinates within bounds of environment
    x = np.random.randint(0,width*100,1) / 100
    y = np.random.randint(0,height*100,1) / 100
    return Point(x, y)


def find_best_Node(scenario, start_Node, new_Node, min_cost=float('inf')): # Find nearby Node in Tree with lowest cost
    best_Node, best_path = None, None # Initalise nearest Node and shortest path as None
    radius = 5 # Radius around which points are considered

     # Recursively check all children
    if start_Node.children: # If Node has children
        for child in start_Node.children: # Iterate over the children nodes
            temp_Node, temp_path, temp_cost = find_best_Node(scenario, child, new_Node, min_cost=min_cost)
            if temp_Node and temp_path.length < radius and temp_cost < min_cost:
                best_Node, best_path, min_cost = temp_Node, temp_path, temp_cost
   
    # CONNECTOR FUNCTION #
    connect_line = LineString([start_Node.point, new_Node.point]) # Create a line that connects to the new Node

    # COLLISION CHECK #
    if scenario.collision_free(connect_line): # If the line does not collide with the environment   
        # Find length of connecting line and cost of this node
        length = connect_line.length
        cost = start_Node.cost

        # If this cost is less than the saved min cost, and within a radius of the new point
        if length < radius and cost < min_cost:
            best_Node = start_Node # set nearest node
            best_path = connect_line # define the shortest path
            min_cost = best_Node.cost # then update minimum cost
    return best_Node, best_path, min_cost


def extract_all_edges(start_Node, total_tree=[]): # Extract all edges from tree
    if start_Node.children: # Iterate over all the children of the start_Node
        for child in start_Node.children: # For each child
            total_tree.append(child.path_to_parent) # Append path to total_tree
            extract_all_edges(child, total_tree) # Recursively repeat over all its children
    return total_tree


def find_nearby_nodes(start_Node, goal_Node, tol, nearby_Nodes=[]): # Find Nodes that are near the goal if several paths exist
    if start_Node.children: # Iterate over all the children of the start_Node
        for child in start_Node.children: # For each child
            if child.point.distance(goal_Node.point) < tol: # If the child is within the tolerated distance of the goal
                nearby_Nodes.append(child) # Add the node to the nearby_Nodes list
            find_nearby_nodes(child, goal_Node, tol, nearby_Nodes) # Recursively repeat over all its children
    return nearby_Nodes


def extract_path(final_Node, path=[]): # Extract only final path from tree
    if final_Node.parent is not None: # As long as there is a parent
        path.append(final_Node.path_to_parent) # Append the path_to_parent to final path
        extract_path(final_Node.parent, path) # Recursively repeat one layer up
    return path
