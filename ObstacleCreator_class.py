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
        Polygon = shapely.geometry.Polygon([(x, y),(x, y + height), (x + height, y + height), (x + height, y), (x, y)])
        self.polygons.append(Polygon)
        return Polygon

    def return_obstacles(self):
        return self.polygons