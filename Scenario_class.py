""" This is the class definition of a scenario in which the planning algorithm is tested.
    Coordinate system used is (x, y), starting in bottom left corner 
"""

import shapely.geometry
import matplotlib.pyplot as plt

class Scenario:
    def __init__(self, env_width, env_height, boundary_collision=False):
        # Set width and height
        self.width = env_width
        self.height = env_height

        if boundary_collision: # If boundary collision is set to true, bbox is created.
            bbox_coords = [(0, 0), (0, self.width), (self.width, self.height), (self.width, 0), (0, 0)]
            self.boundary = shapely.geometry.LineString(bbox_coords)
        else:
            self.boundary = None # Else None boundary
        pass

    def set_start_goal(self, start, goal):
        # Set start and goal, and assert datatype is correct
        assert isinstance(start, shapely.geometry.Point) and isinstance(goal, shapely.geometry.Point), "Start and goal are not defined by points!" 
        self.start, self.goal = start, goal
        pass


    def set_obstacles(self, obstacles):
        # Set list of obstacles, and assert datatype is correct
        assert all(isinstance(x, shapely.geometry.Polygon) for x in obstacles), "Obstacles are not polygons!" 
        self.obstacles = obstacles # define obstacles as list of shapely objects
        pass


    def collision_free(self, object): # Check if given path/points object from RRT is collision_free
        try: # Object can be any shapely object (point, line or polygon)
            object.geom_type # Check if object is shapely geometry object
            for obstacle in self.obstacles:
                if object.intersects(obstacle): # Check collisions with obstacles in scenario
                    return False # Returns False if collision occurs (not collision free)
            return True # Returns True if no collisions
        except AttributeError: # In case object does not have geom_type attribute (not shapely object)
            print("The object is not a shapely object (Point, LineString, Polygon etc.)")


    def plot_scenario(self): # Plot the scenario in matplotlib
        # Set boundaries for drawing scenario
        plt.xlim([0,  self.width])
        plt.ylim([0, self.height])

        # Draw obstacles
        for obstacle in self.obstacles:
            plt.plot(*obstacle.exterior.xy)
        
        # Draw boundary if it exists
        if self.boundary is not None:
            plt.plot(*self.boundary.xy)
        plt.show()
        pass