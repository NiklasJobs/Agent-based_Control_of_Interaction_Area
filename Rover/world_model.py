from shapely import Polygon, MultiPolygon

def obstacle_in_world_model(O1, O2):
    """
    Checks if obstacle O1 is already included in obstacle O2 (world model)
    """
    new_polygon = Polygon(O1)
    for obstacles_in_WM in O2:
        if Polygon(obstacles_in_WM).contains(new_polygon):
            return True
    return False

def check_overlap(O1, O2):
    """
    Checks if obstacle O1 overlaps with obstacle O2 (world model)
    """
    for obstacles_in_WM in O2:
        if isinstance(Polygon(O1).union(Polygon(obstacles_in_WM)), MultiPolygon):
            continue
        else:
            return True
    return False

def merge(O1, O2):
    """
    Merges obstacle O1 with obstacle O2 (world model)
    """
    polygon1 = Polygon(O1)
    polygon2 = Polygon(O2)
    merged_polygon = polygon1.union(polygon2)
    simplified_merged_polygon = merged_polygon.simplify(tolerance=1, preserve_topology=False) #get rid of multiple points on the same line
    merged_polygon_points = list(simplified_merged_polygon.exterior.coords)         
    merged_polygon_points.pop()
    return merged_polygon_points
    
def overlap_index(O1, O2):
    for i, geofences_in_WM in enumerate(O2):
        if isinstance(Polygon(O1).union(Polygon(geofences_in_WM)), MultiPolygon):
            continue
        else:
            return i