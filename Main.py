import numpy as np
import matplotlib.pyplot as plt
from GEOFENCES import geofences
from matplotlib.animation import FuncAnimation
from shapely.geometry import Point, Polygon
from Rover.Rover import Rover
import pygame

#Test Commit

# Definiere die Farben
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
RED = (255, 0, 0)

# Definiere die Größe der Welt
WIDTH = 400
HEIGHT = 400

# Definiere die Zielkoordinate
TARGET_COORDINATE = (50, 20)

# Definiere die Startpositionen der Rover
start_positions = {
    1: (30, 100),
    2: (300, 200),
    3: (380, 320),
    4: (200, 300),
    5: (350, 350)
}

def main():
    pygame.init()

    # Initialisiere das Fenster
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("Rover Simulator")

    # Clock-Objekt zur Steuerung der Framerate
    clock = pygame.time.Clock()

    # Erzeuge eine Liste von Rover-Objekten
    rovers = [Rover(ID, start_positions[ID], TARGET_COORDINATE) for ID in range(1, 4)]

    # Schriftart und Schriftgröße definieren
    font = pygame.font.Font(None, 16)

    # Main loop
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        # Bewege und zeichne die Rover
        screen.fill(WHITE)
        for rover in rovers:
            rover.move()
            rover.draw_rover(screen)
            rover.draw_points(screen)
            rover.draw_path(screen)

        # Zeichne das Ziel (ein Quadrat mit einem "G")
        pygame.draw.rect(screen, BLACK, (TARGET_COORDINATE[0]-6, TARGET_COORDINATE[1]-6, 15, 15))
        text_surface = font.render("G", True, WHITE)
        screen.blit(text_surface, (TARGET_COORDINATE[0]-3, TARGET_COORDINATE[1]-3))

        # Zeichne die Geofences
        for geofence in geofences:
            pygame.draw.polygon(screen, RED, geofence)

        pygame.display.flip()
        clock.tick(10)

    pygame.quit()

if __name__ == "__main__":
    main()