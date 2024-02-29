from world import obstacles
import random
from shapely import Polygon, Point
from vehicle.communication import *
from vehicle.navigation import *
from vehicle.world_model import *

#minimal distance for Rovers to a obstacle
MIN_DISTANCE = 15

# Definiere die Klasse für den Rover
class Rover:
    instantiated_rovers = {}
    
    def __init__(self, id, start_position, target_coordinate, WIDTH, HEIGHT):
        self.id = id
        self.x, self.y = start_position
        #self.instantiated_rovers  = instantiated_rovers 
        self.target_coordinate = target_coordinate
        self.WIDTH = WIDTH
        self.HEIGHT = HEIGHT
        self.radius = 10 #radius for drawing
        self.color = (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255))
        self.move_points = [target_coordinate]  #list of WP that will be used to reach target (initialized with target_coordinat)
        self.target_point_index = 0  
        self.world_model = []  #internal world model of the rover
        self.avoiding_WP = []
        Rover.instantiated_rovers[self.id] = self
        self.counted = False
        self.reached_target = False
        
                
    def move(self):
        if self.move_points:
            current_target_x, current_target_y = self.move_points[0]

            #heading to target
            dx = current_target_x - self.x
            dy = current_target_y - self.y
            #distance to target
            distance_to_target = ((dx ** 2) + (dy ** 2)) ** 0.5

            # observation of the world
            self.obstacle_detection()

            #check if target is reached
            if distance_to_target <= 1:  
                self.x = current_target_x
                self.y = current_target_y
                self.move_points.pop(0)  
                if not self.move_points:
                    #print(f"Rover {self.id} has reached the target!")
                    self.reached_target = True
            else:
                # Bewegung des Rovers zum Ziel
                speed = 0.8  # Bewegungsgeschwindigkeit
                self.x += dx / distance_to_target * speed
                self.y += dy / distance_to_target * speed

    def obstacle_detection(self):
                       
        #detection range
        detection_range = Point(self.x, self.y).buffer(30)

        #check if obstacles are in sensor range
        for obstacle in obstacles:
            if detection_range.intersects(Polygon(obstacle)) and not obstacle_in_world_model(obstacle, self.world_model): #detection of unknown obstacle
                
                # update of world model
                world_model_update(obstacle, self.world_model)

                # update of navigation
                self.move_points = perform_navigation(obstacle, self.world_model, self.x, self.y, self.target_coordinate, self.WIDTH, self.HEIGHT, MIN_DISTANCE)                                                    
                
                # perform communication
                no_communication()
                #full_communication(obstacle, self.instantiated_rovers, self.id)
                #three_receives(obstacle, self.instantiated_rovers, self.id)
                
    def receive_message(self, obstacle, sender):
        if not obstacle_in_world_model(obstacle, self.world_model): #detection of unknown obstacle
                
                # update of world model
                world_model_update(obstacle, self.world_model)

                # update of navigation
                self.move_points = perform_navigation(obstacle, self.world_model, self.x, self.y, self.target_coordinate, self.WIDTH, self.HEIGHT, MIN_DISTANCE)                                                    
                
                 
                 
    '''
    def full_communication(self, obstacle, instantiated_rovers, sender):
        for rover_id, rover in instantiated_rovers.items():
            if rover_id != sender:  # Avoid sending message to itself
                rover.receive_message(obstacle, sender)
    '''