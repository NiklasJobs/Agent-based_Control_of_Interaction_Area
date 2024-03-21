from world.OBSTACLES import obstacles
import random
from shapely import Polygon, Point
from vehicle.rover.utility import *
from vehicle.communication import *
from vehicle.navigation import *
from vehicle.world_model import *


MIN_DISTANCE = 15               # minimal distance for an avoiding WP around an obstacle for Rovers
SPEED = 0.2                     # move speed: 0.1 == 1 m/s == 3,6km/h

# definition of class Rover
class Rover:
    instantiated_rovers = {}
    
    def __init__(self, id, start_position, target_coordinate, WIDTH, HEIGHT, COMM_TYPE):
        self.id = id
        self.x, self.y = start_position
        self.target_coordinate = target_coordinate
        self.WIDTH = WIDTH
        self.HEIGHT = HEIGHT
        self.comm_type = COMM_TYPE
        self.radius = 10                                                                        #radius for drawing
        self.color = (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255))   #colour for drawing
        self.move_points = [target_coordinate]                                                  #list of WP that will be used to reach target (initialized with target_coordinat)
        self.target_point_index = 0  
        self.world_model = []                                                                   #internal world model of the rover including obstacles
        self.avoiding_WP = []
        Rover.instantiated_rovers[self.id] = self
        self.counted = False                                                                    # relevant for counting the rovers, which reached the target
        self.reached_target = False                                                             # relevant for counting the rovers, which reached the target
        self.elapsed_time_to_target = 0
        self.usefull_comms = 0                                                                  # relevant for counting usefull communications
        self.not_usefull_comms = 0                                                              # relevant for counting not usefull communications
        self.moved_distance = 0
        self.distance_to_target = Point(start_position).distance(Point(target_coordinate))
        
                
    def move(self):
        if self.move_points:
            current_target_x, current_target_y = self.move_points[0]

            #heading to target
            dx = current_target_x - self.x
            dy = current_target_y - self.y
                        
            #distance to target
            distance_next_WP = ((dx ** 2) + (dy ** 2)) ** 0.5

            # observation of the world until target is reached
            if self.reached_target == False:
                self.obstacle_detection()
 
            #check if target is reached
            if distance_next_WP <= 1:  
                self.x = current_target_x
                self.y = current_target_y
                self.move_points.pop(0)  
                if not self.move_points and self.reached_target == False:
                    self.reached_target = True
            else:
                # Bewegung des Rovers zum Ziel
                self.x += dx / distance_next_WP * SPEED
                self.y += dy / distance_next_WP * SPEED
                self.moved_distance += (((dx / distance_next_WP * SPEED) ** 2) + ((dy / distance_next_WP * SPEED) ** 2)) ** 0.5
                self.distance_to_target = distance_to_target(self.move_points, self.x, self.y)
                self.estimated_team_utility = utility_estimation(self.distance_to_target)
                       
    def obstacle_detection(self):
        detection_range = Point(self.x, self.y).buffer(50)                              #sensor range

        #check if obstacles are in sensor range
        for obstacle in obstacles:
            if detection_range.intersects(Polygon(obstacle)) and not obstacle_in_world_model(obstacle, self.world_model): #detection of unknown obstacle
                
                # update of world model
                world_model_update(obstacle, self.world_model)
                
                # update of navigation if detected obstacle intersects with current path
                if obstacle_intersects_path(self, obstacle, self.move_points, self.x, self.y, MIN_DISTANCE, test = 0):
                    self.move_points = perform_navigation(obstacle, self.world_model, self.x, self.y, self.target_coordinate, self.WIDTH, self.HEIGHT, MIN_DISTANCE)                                                    
                
                # perform communication
                if self.comm_type == "Full":
                    full_comm(obstacle, self.instantiated_rovers, self.id)
                elif self.comm_type == "Min_Distance":
                    minimum_distance_comm(obstacle, self.instantiated_rovers, self.id, self.x, self.y)
                elif self.comm_type == "Max_Team_Utility":
                    max_utility_comm(obstacle, self.instantiated_rovers, self.id)
                elif self.comm_type == "Direct_Impact":
                    direct_impact_comm()
                elif self.comm_type == "Future_Impact":
                    future_impact_comm()
                else:
                    no_comm()
                                         
                
    def receive_message(self, obstacle, sender):
        if obstacle_in_world_model(obstacle, self.world_model): #reception of already known obstacle
            self.not_usefull_comms += 1
        else:                                                   #reception of unknown obstacle
            # update of world model
            world_model_update(obstacle, self.world_model)

            # update of navigation if percepted obstacle intersects with current path
            if obstacle_intersects_path(self, obstacle, self.move_points, self.x, self.y, MIN_DISTANCE, test = 1):   
                self.move_points = perform_navigation(obstacle, self.world_model, self.x, self.y, self.target_coordinate, self.WIDTH, self.HEIGHT, MIN_DISTANCE)
                self.usefull_comms += 1
            else:  # not usefull communication
                self.not_usefull_comms += 1
            
            