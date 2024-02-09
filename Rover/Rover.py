from GEOFENCES import geofences
import pygame
import random
from shapely import Polygon, Point
from Rover.Geofence_Avoidance import *

# Definiere die Farben
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)

# Definiere die Größe der Welt
WIDTH = 400
HEIGHT = 400

#minimal distance for Rovers to a Geofence
MIN_DISTANCE = 5

# Definiere die Klasse für den Rover
class Rover:
    def __init__(self, id, start_position, target_coordinate):
        self.id = id
        self.target_coordinate = target_coordinate
        self.radius = 10
        self.x, self.y = start_position
        self.color = (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255))
        self.move_points = [target_coordinate]  # Liste der Bewegungspunkte mit der Start- und Zielposition initialisieren
        self.target_point_index = 0  # Index des aktuellen Ziel-Punktes in der Liste
        self.world_model = []  # Liste, die das interne Weltmodell des Rovers darstellt
        self.avoiding_WP = []   #List with possible WP, to go around the geofences
        
    def point_inside_world(self, point):
        """
        Prüft, ob ein Punkt innerhalb der Weltgrenzen liegt.
        :param point: Der Punkt als (x, y) Koordinate.
        :return: True, wenn der Punkt innerhalb der Weltgrenzen liegt, ansonsten False.
        """
        x, y = point
        if 0 <= x <= WIDTH and 0 <= y <= HEIGHT:
            return True
        return False

    def move(self):
        if self.move_points:
            current_target_x, current_target_y = self.move_points[0]

            # Berechnung der Richtung zum Ziel
            dx = current_target_x - self.x
            dy = current_target_y - self.y

            # Entfernung zum Ziel berechnen
            distance_to_target = ((dx ** 2) + (dy ** 2)) ** 0.5

            # Geofence-Erkennung
            self.geofence_detection()

            # Überprüfen, ob das Ziel erreicht ist
            if distance_to_target <= 1:  # Anpassen der Bedingung entsprechend der gewünschten Genauigkeit
                self.x = current_target_x
                self.y = current_target_y
                self.move_points.pop(0)  # Löschen des erreichten Punkts aus der Liste
                if not self.move_points:
                    print(f"Zielpunkt durch Rover {self.id} erreicht!")
            else:
                # Bewegung des Rovers zum Ziel
                speed = 1  # Bewegungsgeschwindigkeit
                self.x += dx / distance_to_target * speed
                self.y += dy / distance_to_target * speed

    def geofence_detection(self):
                       
        # Erstelle einen Kreis um den Rover mit einem Radius von 20 Einheiten
        rover_circle = Point(self.x, self.y).buffer(20)

        # Überprüfe, ob der Kreis einen bekannten Geofence schneidet
        for geofence in geofences:
            geofence_polygon = Polygon(geofence)
            if rover_circle.intersects(geofence_polygon):
                # Wenn der Rover dem Geofence nahe genug kommt, füge ihn zur world_model-Liste hinzu
                if geofence not in self.world_model:
                    self.world_model.append(geofence)
                    #possible_avoiding_WP berechnen
                    #checken ob die possible_avoiding_WP in nicht in self.world_model liegen und innerhalb der Karte sind --> avoiding_WP
                    #A_Star mit den avoiding_WP ausführen
                    #Resultat von A_Star zu move_points hinzufügen
                    self.possible_avoiding_WP = gen_one_point_angle_bisect(geofence, MIN_DISTANCE) 
                    
                    #check if new created possible_avoiding_WP are inside the world --> if yes, apend them to avoiding_WP
                    for point in self.possible_avoiding_WP:
                        if self.point_inside_world(point):
                            self.avoiding_WP.append(point) 
                    
                    #check if resulting avoiding_WP are inside any known geofences of world_model --> if yes, remove them from avoiding_WP
                    self.points_to_remove =[]
                    for point in self.avoiding_WP:
                        for geofence in self.world_model:                        
                            if Polygon(geofence).contains(Point(point)):
                                self.points_to_remove.append(point) 
                    for point in self.points_to_remove:
                        self.avoiding_WP.remove(point)
                    
                    #generate path that avoids the known geofences (world_model) 
                    self.move_points = a_star((self.x, self.y), self.target_coordinate, self.avoiding_WP, self.world_model)
                    

    def draw_points(self, screen):
        """
        Zeichnet alle Punkte aus avoiding_WP auf den Bildschirm.
        :param screen: Die Pygame Surface, auf der gezeichnet wird.
        """
        for point in self.avoiding_WP:
            pygame.draw.circle(screen, BLACK, point, 3)
        
    def draw_rover(self, screen):
        pygame.draw.circle(screen, self.color, (self.x, self.y), self.radius)
        font = pygame.font.Font(None, 20)
        text_surface = font.render(str(self.id), True, WHITE)
        screen.blit(text_surface, (self.x - 5, self.y - 5))

    def draw_path(self, screen):
        # Überprüfen, ob es genügend Punkte gibt, um eine Linie zu zeichnen
        if len(self.move_points) < 1:
            return
        current_position = (int(self.x), int(self.y))
        points_to_draw = [current_position] + self.move_points
        pygame.draw.lines(screen, (255, 0, 0), False, points_to_draw, 2)
    