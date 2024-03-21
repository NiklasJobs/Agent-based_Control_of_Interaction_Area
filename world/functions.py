import random
from shapely.geometry import Point, Polygon

def generate_random_start_positions(num_positions, obstacles, WIDTH, HEIGHT):
    start_positions = []
    obstacle_polygons = [Polygon(obstacle) for obstacle in obstacles]

    while len(start_positions) < num_positions:
        # generate random coordinates
        x = random.randint(0, WIDTH)
        y = random.randint(0, HEIGHT)
        
        # check whether the generated point does not intersect an obstacle
        point = Point(x, y)
        inside_obstacle = False
        for obstacle_polygon in obstacle_polygons:
            if obstacle_polygon.intersects(point):
                inside_obstacle = True
                break
        
        if not inside_obstacle:
            start_positions.append((x, y))
    
    start_positions_dict = {i+1: pos for i, pos in enumerate(start_positions)}
    return start_positions_dict

def generate_random_target_position(obstacles, WIDTH, HEIGHT, min_distance=15):
    obstacle_polygons = [Polygon(obstacle) for obstacle in obstacles]

    while True:
        x = random.randint(0, WIDTH)
        y = random.randint(0, HEIGHT)
        
        point = Point(x, y)
        if all(point.distance(obstacle) >= min_distance for obstacle in obstacle_polygons):
            return x, y
        
def generate_random_target_positions(num_positions, obstacles, WIDTH, HEIGHT, min_distance=15):
    target_positions = []
    obstacle_polygons = [Polygon(obstacle) for obstacle in obstacles]

    while len(target_positions) < num_positions:
        # generate random coordinates
        x = random.randint(0, WIDTH)
        y = random.randint(0, HEIGHT)
        
        # check whether the generated point is far enough away from all obstacles
        point = Point(x, y)
        if all(point.distance(obstacle) >= min_distance for obstacle in obstacle_polygons):
            target_positions.append((x, y))
    
    target_positions_dict = {i+1: pos for i, pos in enumerate(target_positions)}
    return target_positions_dict