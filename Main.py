from world import obstacles
from vehicle.rover.rover import Rover
from vehicle.draw_functions import *
import pygame

# definition of colours
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
RED = (255, 0, 0)

# definition of world size
WIDTH = 600
HEIGHT = 600

# target coordinate
TARGET_COORDINATE = (110, 50)

# rover startpositions
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

    # initialising the screen
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("Rover Simulator")
    clock = pygame.time.Clock()
        
    # number of rovers
    number_of_rovers = 5
    
    number_of_rovers_in_target = 0
    rovers = [Rover(ID, start_positions[ID], TARGET_COORDINATE, WIDTH, HEIGHT) for ID in range(1, number_of_rovers + 1)]
   
    font = pygame.font.Font(None, 16)
    start_time = pygame.time.get_ticks()
    
    # Main loop
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
                
        screen.fill(WHITE)
        
        # move and draw the rovers
        for rover in rovers:
            rover.move()
            draw_rover(rover, screen)
            draw_path(rover, screen)
            #draw_avoiding_WP(rover,screen)
        
        # check if all rovers reached the target
        for rover in rovers:
            if rover.reached_target and not rover.counted:
                number_of_rovers_in_target += 1
                rover.counted = True    
                    
        if number_of_rovers_in_target >= number_of_rovers:
            print("All Rovers have reached the target!")
            elapsed_time = (pygame.time.get_ticks() - start_time) / 1000  
            print("Eleapsed Time: ", elapsed_time)
            running = False

        # draw the target
        pygame.draw.rect(screen, BLACK, (TARGET_COORDINATE[0]-6, TARGET_COORDINATE[1]-6, 20, 20))
        text_surface = font.render("G", True, WHITE)
        screen.blit(text_surface, (TARGET_COORDINATE[0]-3, TARGET_COORDINATE[1]-3))

        # draw the world with obstacles
        for obstacle in obstacles:
            pygame.draw.polygon(screen, RED, obstacle)

        pygame.display.flip()
        clock.tick(100)

    pygame.quit()

if __name__ == "__main__":
    main()