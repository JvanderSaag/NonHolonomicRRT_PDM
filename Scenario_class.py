""" This is the class definition of a scenario in which the planning algorithm is tested.
    Coordinate system used is (x, y), starting in bottom left corner 
"""

import shapely.geometry
import matplotlib.pyplot as plt
import matplotlib.patches

class Scenario:
    def __init__(self, env_width, env_height, boundary_collision=False):
        # Set width and height
        self.width = env_width
        self.height = env_height

        # Initalise attributes as none, updated with other functions
        self.obstacles, self.goal, self.start, self.path, self.total_tree = [], None, None, [], []

        # If boundary collision is set to true, bbox is created.
        if boundary_collision: 
            bbox_coords = [(0, 0), (0, self.width), (self.width, self.height), (self.width, 0), (0, 0)]
            self.boundary = shapely.geometry.LineString(bbox_coords)
        else:
            self.boundary = None # Else None boundary
        pass


    def set_start_goal(self, start, goal):
        # Set start and goal, and assert datatype is correct
        assert isinstance(start, shapely.geometry.Point) and isinstance(goal, shapely.geometry.Point), "AssertError: Start and goal are not defined by points!" 
        self.start, self.goal = start, goal
        pass


    def set_obstacles(self, obstacles):
        # Set list of obstacles, and assert datatype is correct
        assert all(isinstance(x, shapely.geometry.Polygon) for x in obstacles), "AssertError: Obstacles are not polygons!" 
        self.obstacles = obstacles # define obstacles as list of shapely objects
        pass
    

    def set_path(self, path):
        # Include path from motion planner into scenario class
        self.path = path
        pass

    
    def set_totaltree(self, tree):
        self.total_tree = tree
        pass 


    def collision_free(self, object): # Check if given path/points object from RRT is collision_free
        try: # Object can be any shapely object (point, line or polygon)
            object.geom_type # Check if object is shapely geometry object
            for obstacle in self.obstacles:
                if object.intersects(obstacle): # Check collisions with obstacles in scenario
                    return False # Returns False if collision occurs (not collision free)
            if self.boundary is not None:
                if object.intersects(self.boundary): # Check collisions with the boundary in scenario
                    return False # Returns False if collision occurs (not collision free)
            return True # Returns True if no collisions
        
        except AttributeError: # In case object does not have geom_type attribute (not shapely object)
            print("AttributeError: The object is not a shapely object (Point, LineString, Polygon etc.)")
        pass
    
    # Return coordinates of obstacles, path, start and goal in that order
    # Coordinate system is x, y starting from bottom left
    def return_coordinates(self):
        # Get obstacle coordinates, a list of lists, where each nested list represents the exterior bounds of one obstacle
        obstacle_coords = []
        for obstacle in self.obstacles:
            obstacle_coords.append(obstacle.exterior.coords[:])

        # Get the coordinats of the path, return
        path_coords = []
        for segment in self.path[::-1]:
            path_coords.append(segment.coords[0])

        return obstacle_coords, path_coords, self.start.coords[:][0], self.goal.coords[:][0]


    def return_scenariosize(self): # Returns size of environment (width, height)
        return (self.width, self.height)


    def plot_scenario(self, plot_all_trees=False): # Plot the scenario in matplotlib
        fig, ax = plt.subplots()

        # Set boundaries for drawing scenario
        plt.xlim([0, self.width])
        plt.ylim([0, self.height])

        # Draw obstacles
        for obstacle in self.obstacles:
            ax.add_patch(matplotlib.patches.Polygon(obstacle.exterior.coords, color="grey"))
        
        # Draw start and goal
        if self.start is not None and self.goal is not None:
            plt.scatter(self.start.x, self.start.y, s=50, c='g', marker='o', label='Start')
            plt.scatter(self.goal.x, self.goal.y, s=60, c='r', marker='*', label='Goal')

        # Draw boundary, if it exists
        if self.boundary is not None:
            plt.plot(*self.boundary.xy, color="k")
        
        # Draw path, if it exists and not all trees are plotted
        if not plot_all_trees:
            for path in self.path:
                plt.plot(*path.xy, c='tab:blue', alpha=0.4)
                

        # Draw all trees if it plot_all_trees is True
        if plot_all_trees:
            for path in self.path:
                plt.plot(*path.xy, c='tab:blue', alpha=0.5)
            for tree in self.total_tree:
                plt.plot(*tree.xy, c='tab:green', alpha=0.15)

        plt.legend()
        plt.show()
        pass