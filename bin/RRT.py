""" These are the functions used for the RRT* algorithm, running the main RRT() function will
    update the scenario class with the final path. Step size and distance tolerance can both be changed when calling RRT().
"""

import numpy as np
from shapely.geometry import Point, LineString
from tqdm import tqdm
from bin.Reeds_Shepp_Curves import reeds_shepp_path_planning

class TreeNode: # Tree Node class that RRT uses
    def __init__(self, point, yaw):
        self.point = point # This should be shapely PointObject, containing coordinate informatioj
        self.yaw = yaw # Orientation of vehicle

        self.cost = 0 # Initial cost of zero
        self.parent = None # Children, list of nodes
        self.path_to_parent = None # Path from parent, list of paths
        self.children = [] # Children, only used to draw the entire RRT tree


## RRT/ RRT* implementation for global motion planner ##
def RRT(N_iter, scenario, step_size=float('inf'), dist_tolerance=1, star=True, non_holonomic=True, force_plot_tree=False):
    # Initialise start and goal node
    start_Node, goal_Node = TreeNode(scenario.start[0], scenario.start[1]), TreeNode(scenario.goal[0], scenario.goal[1])

    for n in tqdm(range(N_iter)): # Max N_iter iterations
        if np.random.random_sample() < 0.05: # Have a chance of picking the goal node as the sampled node
            sampled_Node = goal_Node
        elif np.random.random_sample() < 0.05: # Have a chance of changing orientation to face goal
            random_point = rand_coords(scenario.width, scenario.height)
            angle_to_goal = np.degrees(np.arctan((goal_Node.point.y - random_point.y) / (goal_Node.point.x - random_point.x))) + 180
            sampled_Node = TreeNode(random_point, angle_to_goal)
        else:  # Otherwise, randomly sample a point in the environment
            sampled_Node = TreeNode(rand_coords(scenario.width, scenario.height), np.deg2rad(np.random.randint(-180, 180,1)))

        # If the sampled point collides with obstacles
        if not scenario.collision_free(sampled_Node.point):
            continue # Continue to next iteration

        # Find the closest node to sampled point
        parent_Node, path_to_parent, _ = find_closest_Node(scenario, start_Node, sampled_Node, non_holonomic)

        if parent_Node is not None and not parent_Node.point.equals(sampled_Node.point):  # If a nearest node is found     
            if path_to_parent.length > step_size: # In the case that this parent node is not within the step size
                # Find a new point on the connecting line that is within the radius
                new_Node_inradius = TreeNode(path_to_parent.interpolate(step_size), parent_Node.yaw)

                # Update the sampled Node to this new point
                sampled_Node = new_Node_inradius
                        
            # Update the sampled_Node cost
            sampled_Node.cost = parent_Node.cost + path_to_parent.length 
            if star: # If RRT* is used, the tree will be redrawn
                # Within a certain radius, find nearby points
                radius = 10
                Nodes_near_sample = find_nearby_nodes(start_Node, sampled_Node, radius, nearby_Nodes=[])

                if Nodes_near_sample: # If there are nearby nodes
                    min_cost = sampled_Node.cost # First set the upper bound of cost (current cost of sampled Node)
                    
                    for node in Nodes_near_sample: # For each nearby node
                        # Estimate cost by using distance and difference in angle, avoids redrawing new connectors for each node
                        cost_estimate = sampled_Node.point.distance(node.point) + min(abs(sampled_Node.yaw - node.yaw), 360 - abs(sampled_Node.yaw - node.yaw))
                        cost_via_node = node.cost + cost_estimate

                        if cost_via_node < min_cost: # If this cost is less, update the parent Node
                            parent_Node, min_cost = node, cost_via_node
                            sampled_Node.cost = min_cost

            # Create the path to parent and check if it is collision-free
            path_to_parent = create_connector(parent_Node, sampled_Node, scenario, non_holonomic) # NEW CONNETOR FUNCTION
            if path_to_parent is None:
                continue
            if not scenario.collision_free(path_to_parent):
                continue # If it collides, continue to next iteration

            # Update parent node with children, and sampled Node with parent
            parent_Node.children.append(sampled_Node)
            sampled_Node.parent = parent_Node 
            sampled_Node.path_to_parent = path_to_parent

            # Early stop if normal RRT is used, as once the goal is reached the path won't change
            if not star and sampled_Node.point.distance(goal_Node.point) < dist_tolerance:
                final_path = extract_path(sampled_Node)
                total_tree = extract_all_edges(start_Node)
                
                # Update path and tree attributes of class
                scenario.set_path(final_path)
                scenario.set_totaltree(total_tree)

                print(f"\nRRT finished within {n} iterations")
                return
    
    # Find the nodes near the goal
    Nodes_near_goal = find_nearby_nodes(start_Node, goal_Node, dist_tolerance)

    if Nodes_near_goal:
        # Else RRT* has found a path, find the shortest one if there are several
        #cost_estimate = sampled_Node.point.distance(node.point) + min(abs(sampled_Node.yaw - node.yaw), 360 - abs(sampled_Node.yaw - node.yaw))
        Node_min_cost = min(Nodes_near_goal, key=lambda x: x.cost + 3 * sampled_Node.point.distance(x.point) + min(abs(sampled_Node.yaw - x.yaw), 360 - abs(sampled_Node.yaw - x.yaw)))
        shortest_path = extract_path(Node_min_cost)

        # Finally set final path and 'total' tree containing all edges
        final_path = shortest_path 
        total_tree = extract_all_edges(start_Node)

        # Update class attributes
        scenario.set_path(final_path)
        scenario.set_totaltree(total_tree)

        # RRT* is done
        print(f"\nRRT* finished within {n+1} iterations")
        return

    else: # If it does not converge, there will be no Nodes near the goal
        if force_plot_tree: # Force plot the tree, regardless whether it converges
            print("Force plotted the entire tree, convergence not guaranteed")
            scenario.set_totaltree(extract_all_edges(start_Node))
            return 
        # Raise exception if not force plotting
        raise Exception("\nRRT could not find a suitable path within the given number of iterations. Please try again.\nIf it consistently fails to complete, increase the number of iterations.")
    

def rand_coords(width, height): # Generate random coordinates within bounds of environment
    x = np.random.randint(0,width*100,1) / 100
    y = np.random.randint(0,height*100,1) / 100
    return Point(x, y)


def find_closest_Node(scenario, start_Node, new_Node, non_holonomic, min_length=float('inf')): # Find closest Node in Tree
    nearest_Node, shortest_path = None, None # Initalise nearest Node and shortest path as None

    # Recursively check all children
    if start_Node.children: # If Node has children
        for child in start_Node.children: # Iterate over the children nodes
            temp_Node, temp_path, temp_length = find_closest_Node(scenario, child, new_Node, non_holonomic, min_length=min_length)
            if temp_Node is None: # If the node does not exist
                return nearest_Node, shortest_path, min_length # Return current best Node and path
            if temp_length < min_length:
                nearest_Node, shortest_path, min_length = temp_Node, temp_path, temp_length
    
    # Either there are no children, or the recursive search is done (following code will be reached) #

    # Create connector that connects the two Nodes, returns None if the connector collides with environment
    connect_line = create_connector(start_Node, new_Node, scenario, non_holonomic)
    if connect_line is None:
        return nearest_Node, shortest_path, min_length # Return current best Nodes and path
    
    length = connect_line.length # Find length of connecting line

    # If this length is less than the saved min, we can once again update the nearest node
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


def find_nearby_nodes(start_Node, goal_Node, tol, nearby_Nodes=[]): # Find Nodes that are near the goal if several paths exist
    if start_Node.children: # Iterate over all the children of the start_Node
        for child in start_Node.children: # For each child
            if child.point.distance(goal_Node.point) < tol: # If the child is within the tolerated distance of goal node
                nearby_Nodes.append(child) # Add the node to the nearby_Nodes list
            find_nearby_nodes(child, goal_Node, tol, nearby_Nodes) # Recursively repeat over all its children
    return nearby_Nodes


def extract_path(final_Node, path=[]): # Extract only final path from tree
    if final_Node.parent is not None: # As long as there is a parent
        path.append(final_Node.path_to_parent) # Append the path_to_parent to final path
        extract_path(final_Node.parent, path) # Recursively repeat one layer up
    return path


def create_connector(Node1, Node2, scenario, non_holonomic=True):
    # If linear connectors are desired, non_holonomic can be set to False
    if not non_holonomic:
        return LineString([Node1.point, Node2.point])
    
    maxc = scenario.curve_radius # Turning radius
    sx, sy, syaw = Node1.point.x, Node1.point.y , Node1.yaw # Coordinates and orientation of Node 1
    gx, gy, gyaw = Node2.point.x, Node2.point.y, Node2.yaw # Coordinates and orientatio of Node 2
    
    # Return list of possible connecting curves from Reeds-Schepp
    connect_line_list = reeds_shepp_path_planning(sx, sy, syaw, gx, gy, gyaw, maxc)
    if connect_line_list is not None: # If list is not empty
        for connect_line in connect_line_list:
            if scenario.collision_free(connect_line): # If the line does not collide with the environment
                return connect_line
    return None # If it does collide with the environment, return None
