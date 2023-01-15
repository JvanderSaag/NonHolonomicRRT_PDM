""" This function creates obstacles for the Scenario class as Shapely objects.
    Authored by Jelmer van der Saag and Hugo Chassagnette for the course RO47005 at TU Delft.
"""
import shapely.geometry

class ObstacleCreator:
    def __init__(self):
        self.polygons = []
        pass

    def create_polygon(self, coords): # ie: coords = [(5,6),(7,1), (8,2),(7,5)]
        Polygon = shapely.geometry.Polygon(coords)
        self.polygons.append(Polygon)
        return Polygon

    def create_rectangle(self, width, height, left_corner):
        x, y = left_corner
        Polygon = shapely.geometry.Polygon([(x, y),(x, y + height), (x + width, y + height), (x + width, y)])
        self.polygons.append(Polygon)
        return Polygon

    def return_obstacles(self):
        return self.polygons