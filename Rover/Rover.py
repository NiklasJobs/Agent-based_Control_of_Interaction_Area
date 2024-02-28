from world import obstacles
import pygame
import random
from shapely import Polygon, Point
from Rover.communication import *
from Rover.navigation import *
from Rover.world_model import *

# Definiere die Farben
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)

#minimal distance for Rovers to a obstacle
MIN_DISTANCE = 15

# Definiere die Klasse für den Rover
class Rover:
    def __init__(self, id, start_position, target_coordinate, WIDTH, HEIGHT):
        self.id = id
        self.target_coordinate = target_coordinate
        self.radius = 10 #radius for drawing
        self.x, self.y = start_position
        self.color = (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255))
        self.move_points = [target_coordinate]  #list of WP that will be used to reach target (initialized with target_coordinat)
        self.target_point_index = 0  
        self.world_model = []  #internal world model of the rover
        self.avoiding_WP = []
        self.WIDTH = WIDTH
        self.HEIGHT = HEIGHT
                
    def move(self):
        if self.move_points:
            current_target_x, current_target_y = self.move_points[0]

            #heading to target
            dx = current_target_x - self.x
            dy = current_target_y - self.y
            #distance to target
            distance_to_target = ((dx ** 2) + (dy ** 2)) ** 0.5

            # obstacle detection
            self.obstacle_detection()

            #check if target is reached
            if distance_to_target <= 1:  
                self.x = current_target_x
                self.y = current_target_y
                self.move_points.pop(0)  
                if not self.move_points:
                    print(f"Zielpunkt durch Rover {self.id} erreicht!")
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
                #world model update
                if check_overlap(obstacle, self.world_model):
                    self.i = overlap_index(obstacle, self.world_model)
                    self.world_model[self.i] = merge(obstacle, self.world_model[self.i])
                else:
                    self.world_model.append(obstacle)
                #end of world model update
                                                    
                #navigation
                self.possible_avoiding_WP =[]
                for obstacle in self.world_model:
                    self.possible_avoiding_WP.extend(avoiding_WP_generation(obstacle, MIN_DISTANCE))    #possible_avoiding_WP berechnen
                self.avoiding_WP = []   #list with possible WP, to go around the obstacles
                for point in self.possible_avoiding_WP:
                    if point_inside_world(point, self.WIDTH, self.HEIGHT):
                        self.avoiding_WP.append(point)  #WP which are inside the world are used
                self.points_to_remove =[]
                for point in self.avoiding_WP:
                    for obstacle in self.world_model:                        
                        if Polygon(obstacle).contains(Point(point)):
                            self.points_to_remove.append(point) 
                for point in self.points_to_remove:
                    self.avoiding_WP.remove(point)      #WP that are inside an obstacle of WorldModel are removed
                self.move_points = a_star((self.x, self.y), self.target_coordinate, self.avoiding_WP, self.world_model) #generate path that avoids the known obstacles (world_model) 
                #end of Navigation
                
                #communication
                communication_to_other_rovers()
                
                #end of communication
                 
         
    def draw_points(self, screen):
        for point in self.avoiding_WP:
            pygame.draw.circle(screen, BLACK, point, 3)
        
    def draw_rover(self, screen):
        pygame.draw.circle(screen, self.color, (self.x, self.y), self.radius)
        font = pygame.font.Font(None, 20)
        text_surface = font.render(str(self.id), True, WHITE)
        screen.blit(text_surface, (self.x - 5, self.y - 5))

    def draw_path(self, screen):
        if len(self.move_points) < 1:
            return
        current_position = (int(self.x), int(self.y))
        points_to_draw = [current_position] + self.move_points
        pygame.draw.lines(screen, (255, 0, 0), False, points_to_draw, 2)
    