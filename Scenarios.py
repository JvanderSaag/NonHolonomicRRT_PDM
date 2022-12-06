import shapely.geometry
import matplotlib.pyplot as plt

class create_obstacles:
    def __init__(self):
        self.polygons = []
        pass
    
    def square_boundary(self, x,y):
        coords = [(0, 0), (0, y), (x, y), (x,0), (0,0)]
        self.Boundary = shapely.geometry.LineString(coords)
        return self.Boundary

    def special_boundary(self, coords):# ie: coords = [(5,6),(7,1), (8,2),(7,5), (5,6)] first and last point must be same
        self.Boundary = shapely.geometry.LineString(coords)
        return self.Boundary

    def polygon(self, coords): # ie: coords = [(5,6),(7,1), (8,2),(7,5)]
        Polygon = shapely.geometry.Polygon(coords)
        self.polygons.append(Polygon)
        return Polygon

    def rectangle(self, width, height, left_corner):
        x, y = left_corner
        Polygon = shapely.geometry.Polygon([x, y],[x, y + height], [x + height, y + height], [x + height, y], [x, y])
        self.polygons.append(Polygon)
        return Polygon

    def Boundary_collision_check(self, geom):
        return geom.intersects(self.Boundary)

    def plotting(self):
        plt.plot(*self.Boundary.xy)
        for geom in self.polygons:
            plt.plot(*geom.exterior.xy)
        plt.show()