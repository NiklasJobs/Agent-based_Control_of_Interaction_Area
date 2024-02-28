from world import obstacles
from Rover.rover import Rover
import pygame

# Definiere die Farben
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
RED = (255, 0, 0)

# definition of world size
WIDTH = 600
HEIGHT = 600

# Definiere die Zielkoordinate
TARGET_COORDINATE = (110, 50)

# Definiere die Startpositionen der Rover
start_positions = {
    1: (150, 170),
    2: (520, 310),
    3: (250, 450),
    4: (480, 440),
    5: (480, 560),
    6: (40, 550),
    7: (30, 40),
    8: (90, 330),
    9: (390, 130),
    10: (570, 580)
}

def main():
    pygame.init()

    # Initialisiere das Fenster
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("Rover Simulator")

    # Clock-Objekt zur Steuerung der Framerate
    #clock = pygame.time.Clock()

    # Erzeuge eine Liste von Rover-Objekten
    rovers = [Rover(ID, start_positions[ID], TARGET_COORDINATE, WIDTH, HEIGHT) for ID in range(1, 11)]

    # Schriftart und Schriftgröße definieren
    font = pygame.font.Font(None, 16)

    # Main loop
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
        screen.fill(WHITE)
        
        # Bewege und zeichne die Rover
        for rover in rovers:
            rover.move()
            rover.draw_rover(screen)
            #rover.draw_points(screen)
            rover.draw_path(screen)

        # Zeichne das Ziel (ein Quadrat mit einem "G")
        pygame.draw.rect(screen, BLACK, (TARGET_COORDINATE[0]-6, TARGET_COORDINATE[1]-6, 20, 20))
        text_surface = font.render("G", True, WHITE)
        screen.blit(text_surface, (TARGET_COORDINATE[0]-3, TARGET_COORDINATE[1]-3))

        #draw the world with obstacles
        for obstacle in obstacles:
            pygame.draw.polygon(screen, RED, obstacle)

        pygame.display.flip()
        #clock.tick(10)

    pygame.quit()

if __name__ == "__main__":
    main()