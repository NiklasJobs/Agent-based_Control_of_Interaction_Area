import heapq
import numpy as np
from shapely.geometry import Point, LineString, Polygon

def gen_one_point_angle_bisect(geofence_corners, min_distance):
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

    :return: The resulting path as a list of waypoints [(x1, y1), (x2, y2), ...].
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
        return total_path

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



