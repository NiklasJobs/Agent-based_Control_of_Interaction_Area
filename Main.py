from world.OBSTACLES import obstacles
from world.functions import *
from vehicle.rover.rover import Rover
from vehicle.draw_functions import *
from output_logger.functions import write_to_file
import pygame

# definition of colours
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
RED = (255, 0, 0)

# definition of world size
WIDTH = 600
HEIGHT = 600

# rover specifics
NUMBER_OF_ROVERS = 6
COMM_TYPE = "No_Comm"                  # choices: No_Comm, Min_Distance, Max_Team_Utility, Direct_Impact, Future_Impact, Full 

# number of simulation trials
TRIALS = 10

# internal simulation time step in seconds
TIME_STEP = 0.01                        
simulation_time = 0

def main():
    pygame.init()

    # initialising the screen
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("Rover Simulator")
    clock = pygame.time.Clock()
    font = pygame.font.Font(None, 16)
    

    rover_times = [[] for i in range(NUMBER_OF_ROVERS)]
    for N in range(TRIALS):
        # target coordinate
        TARGET_COORDINATES = {1:(110, 50), 2:(110, 50), 3:(110, 50), 4:(110, 50), 5:(110, 50), 6:(110, 50), 7:(110, 50), 8:(110, 50), 9:(110, 50), 10:(110, 50)}
        #TARGET_COORDINATE = generate_random_target_position(obstacles, WIDTH, HEIGHT)
        #TARGET_COORDINATES = generate_random_target_positions(NUMBER_OF_ROVERS, obstacles, WIDTH, HEIGHT)
        
        # rover startpositions (fixed or random)
        #START_POSITIONS = {1: (125, 50), 2: (145, 50), 3: (250, 450), 4: (480, 440), 5: (480, 560), 6: (40, 550), 7: (30, 40), 8: (90, 330), 9: (390, 130), 10: (570, 580)}
        #START_POSITIONS = {1: (150, 170), 2: (520, 310), 3: (250, 450), 4: (480, 440), 5: (480, 560), 6: (40, 550), 7: (30, 40), 8: (90, 330), 9: (390, 130), 10: (570, 580)}
        START_POSITIONS = generate_random_start_positions(NUMBER_OF_ROVERS, obstacles, WIDTH, HEIGHT)
        
        start_time = pygame.time.get_ticks()
                    
        # initialising count variables
        simulation_time = 0                           
        number_of_rovers_in_target = 0
        usefull_comms = 0
        not_usefull_comms = 0
        
        # generating rover instances
        rovers = [Rover(ID, START_POSITIONS[ID], TARGET_COORDINATES[ID], WIDTH, HEIGHT, COMM_TYPE) for ID in range(1, NUMBER_OF_ROVERS + 1)]
                    
        # Main loop
        running = True
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False 
            screen.fill(WHITE) 
            
            # Draw the target and obstacles
            for obstacle in obstacles:
                pygame.draw.polygon(screen, RED, obstacle)
            
            # move and draw the rovers and their targets
            
            for rover in rovers:
                pygame.draw.rect(screen, BLACK, (TARGET_COORDINATES[rover.id][0]-6, TARGET_COORDINATES[rover.id][1]-6, 20, 20))
                text_surface = font.render("G", True, WHITE)
                screen.blit(text_surface, (TARGET_COORDINATES[rover.id][0]-3, TARGET_COORDINATES[rover.id][1]-3))
                rover.move()
                draw_rover(rover, screen)
                draw_path(rover, screen)
                # draw_avoiding_WP(rover, screen)

                # Check if rover has reached the target
                if rover.reached_target and not rover.counted:
                    number_of_rovers_in_target += 1
                    rover.counted = True
                    rover.elapsed_time_to_target = simulation_time
                    print(f"Elapsed simulated time until Rover {rover.id} has reached the target: {simulation_time:.1f}s")
                    
                    # Check if all rovers have reached the target
                    if number_of_rovers_in_target >= NUMBER_OF_ROVERS:
                        for rover in rovers:
                            usefull_comms += rover.usefull_comms 
                            not_usefull_comms += rover.not_usefull_comms
                        print("All Rovers have reached the target!")
                        elapsed_time = (pygame.time.get_ticks() - start_time) / 1000  
                        print(f"Elapsed real time: {elapsed_time:.1f}s")
                        print("Number of useful communications: ", usefull_comms)
                        print("Number of not useful communications: ", not_usefull_comms)
                        running = False          
            simulation_time += TIME_STEP                            
        
            pygame.display.flip()
            clock.tick(50)
        for rover in rovers:
                rover_times[rover.id - 1].append(round(rover.elapsed_time_to_target))
    write_to_file(COMM_TYPE, rover_times, 'rover_times.csv')
    pygame.quit()

if __name__ == "__main__":
    main()