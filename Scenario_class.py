""" This is the class definition of a scenario in which the planning algorithm is tested
    Coordinate system used is (x, y), starting in bottom left corner 
"""

from shapely.geometry import Polygon, Point
import matplotlib.pyplot as plt

class Scenario:
    def __init__(self, obstacles, start, goal):
        # Set list of obstacles, and assert datatype is correct
        assert all(isinstance(x, Polygon) for x in obstacles), "Obstacles are not polygons!" 
        self.obstacles = obstacles # define obstacles as list of shapely objects

        # Set start and goal, and assert datatype is correct
        assert isinstance(start, Point) and isinstance(goal, Point), "Start and goal are not defined by points!" 
        self.start, self.goal = start, goal

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
        plt.plot(*self.Boundary.xy)
        for obstacle in self.obstacles:
            plt.plot(*obstacle.exterior.xy)
        plt.show()
