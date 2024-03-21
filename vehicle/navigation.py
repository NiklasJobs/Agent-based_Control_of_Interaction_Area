import heapq
import numpy as np
from shapely.geometry import LineString, Polygon, Point

def obstacle_intersects_path(rover, obstacle, move_points, x, y, MIN_DISTANCE, test):
    current_position = (x, y)
    if move_points == []:
        print("WARNING! Empty move_points!")
        print("Rover ID: ", rover.id)
        print("Rover x-position: ", rover.x)
        print("Rover y-position: ", rover.y)
        print("Rover reached_target value: ", rover.reached_target)
        print("Obstacle: ", obstacle)
        if test == 0:
            print("obstacle_intersects_path wurde aus der obstacle_detection Methode aufgerufen: test=", test)
        else:
            print("obstacle_intersects_path wurde aus der receive_message Methode aufgerufen: test=", test)
        return False
    
    current_path = LineString([current_position] + move_points)
    obstacle_polygon = Polygon(avoiding_WP_generation(obstacle, MIN_DISTANCE))
    if obstacle_polygon.intersects(current_path):
        return True
    else:
        return False

def perform_navigation(obstacle, world_model, x, y, target_coordinate, WIDTH, HEIGHT, MIN_DISTANCE):
    possible_avoiding_WP =[]
    for obstacle in world_model:
        possible_avoiding_WP.extend(avoiding_WP_generation(obstacle, MIN_DISTANCE))    #possible_avoiding_WP berechnen
    avoiding_WP = []   #list with possible WP, to go around the obstacles
    for point in possible_avoiding_WP:
        if point_inside_world(point, WIDTH, HEIGHT):
            avoiding_WP.append(point)  #WP which are inside the world are used
    points_to_remove =[]
    for point in avoiding_WP:
        for obstacle in world_model:                        
            if Polygon(obstacle).contains(Point(point)):
                points_to_remove.append(point) 
    for point in points_to_remove:
        avoiding_WP.remove(point)      #WP that are inside an obstacle of WorldModel are removed
    move_points = a_star((x, y), target_coordinate, avoiding_WP, world_model) #generate path that avoids the known obstacles (world_model) 
    return move_points
    

def point_inside_world(point, WIDTH, HEIGHT):
    """
    Prüft, ob ein Punkt innerhalb der Weltgrenzen liegt.
    :param point: Der Punkt als (x, y) Koordinate.
    :return: True, wenn der Punkt innerhalb der Weltgrenzen liegt, ansonsten False.
    """
    x, y = point
    if 0 <= x <= WIDTH and 0 <= y <= HEIGHT:
        return True
    return False

def avoiding_WP_generation(geofence_corners, min_distance):
    """
    Generate one new point at each corner of the geofence so that the point is placed on the bisecting angle.
    :param geofence_corners: The vertices of the geofence as a list of tuples [(x1, y1), (x2, y2), ...].
    :param min_distance: The distance from the geofence corners to the new points.
    :return: A list of new points in the format [(x1, y1), (x2, y2), ...].
    """
    new_points = []

    num_corners = len(geofence_corners)
    for i in range(num_corners):
        # Get the current corner and the previous and next corners to form angle bisector
        current_corner = np.array(geofence_corners[i])
        previous_corner = np.array(geofence_corners[(i - 1) % num_corners])
        next_corner = np.array(geofence_corners[(i + 1) % num_corners])

        # Compute the vectors from the current corner to the previous and next corners
        vector_previous = previous_corner - current_corner
        vector_next = next_corner - current_corner

        # Calculate the bisecting angle between the two vectors (using exterior angle bisect)
        bisecting_vector = vector_next / np.linalg.norm(vector_next) + vector_previous / np.linalg.norm(vector_previous)
        bisecting_vector /= np.linalg.norm(bisecting_vector)

        # Compute the new point at a distance of min_distance along the bisecting vector
        new_point = tuple((current_corner - min_distance * bisecting_vector).tolist())

        new_points.append(new_point)

    return new_points


def a_star(start, goal, list_of_WP, world_model):
    """
    A* algorithm implementation for a graph with geofences.

    :param start: The starting node (x, y).
    :param goal: The goal node (x, y).
    :param list_of_WP: A list of waypoints (WP), each WP can be reached from any other WP [(x1, y1), (x2, y2), ...].
    :param world_model: A list of geofences where each geofence is defined by its corners.

    :return: The resulting path as a list of waypoints including start as first item [(x1, y1), (x2, y2), ...].
    """

    def heuristic(a, b):
        """
        Calculate the Euclidean distance between two points.
        """
        return np.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

    def path_intersects_world_model(a, b, world_model):
        """
        Check if the path from a to b intersects any geofence from world_model.
        """
        line = LineString([a, b])
        for geofence in world_model:
            if line.intersects(Polygon(geofence)):
                return True
        return False

    def reconstruct_path(came_from, current):
        """
        Reconstruct the path from the start node to the current node.
        """
        total_path = [current]
        while current in came_from:
            current = came_from[current]
            total_path.insert(0, current)
        return total_path[1:]

    open_set = []
    heapq.heappush(open_set, (0, start))

    came_from = {}
    all_nodes = set(list_of_WP) | {start, goal}
    g_score = {node: float('inf') for node in all_nodes}
    g_score[start] = 0

    f_score = {node: float('inf') for node in all_nodes}
    f_score[start] = heuristic(start, goal)

    while open_set:
        current = heapq.heappop(open_set)[1]

        if current == goal:
            path = reconstruct_path(came_from, current)
            return path  # Return the reconstructed path

        for neighbor in all_nodes:
            if neighbor == current or path_intersects_world_model(current, neighbor, world_model):
                continue

            tentative_g_score = g_score[current] + heuristic(current, neighbor)

            if tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                if neighbor not in [i[1] for i in open_set]:
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))

    return []  # Return an empty list if no path is found

def distance_to_target(move_points, x, y):
    total_distance = 0
            
    if len(move_points) == 1:
        total_distance = ((x - move_points[0][0]) ** 2 + (y - move_points[0][1]) ** 2) ** 0.5
    else:
        total_distance = ((x - move_points[0][0]) ** 2 + (y - move_points[0][1]) ** 2) ** 0.5
        for i in range(len(move_points) - 1):
            distance = ((move_points[i+1][0] - move_points[i][0]) ** 2 + (move_points[i+1][1] - move_points[i][1]) ** 2) ** 0.5
            total_distance += distance
    
    return total_distance
