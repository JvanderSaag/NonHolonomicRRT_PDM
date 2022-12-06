import shapely.geometry
import matplotlib.pyplot as plt

coords = [(0, 0), (0, 10), (10, 10), (10,0),(0,0)]
Boundary = shapely.geometry.LineString(coords)
polygon1 = shapely.geometry.Polygon([(5,6),(7,1), (8,2),(7,5)])
polygon2 = shapely.geometry.Polygon([(1,4),(2,2), (3,1),(1,4)])
polys = shapely.geometry.GeometryCollection([polygon1, polygon2])

for geo in polys.geoms:
    if geo.intersects(Boundary):
        print("interstion")

plt.plot(*Boundary.xy)
for geo in polys.geoms:
    x,y = geo.exterior.xy
    plt.plot(x,y)
plt.show()
