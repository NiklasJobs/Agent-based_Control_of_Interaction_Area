from constants import *
from world.obstacles import OBSTACLES
from world.functions import *
from vehicle.rover.rover import Rover
from vehicle.draw_functions import *
from output_logger.functions import *
import pygame
import sys
import time
 
time.sleep(2)
simulation_time = 0 
def main():
    pygame.init()
 
    # initialising the screen
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("Rover Simulator")
    clock = pygame.time.Clock()
    font = pygame.font.Font(None, 16)
    
 
    number_of_communication =[0]*CYCLES  # number of sent messages per Cycle
    total_number_of_communication = 0   # total number of sent messages over all cycles
    rover_times_delta = [[[] for _ in range(NUMBER_OF_ROVERS)] for _ in range(CYCLES)]         # list to store the delta between target time and time to reach target for every simulation cycle
    priorities = [[[] for _ in range(NUMBER_OF_ROVERS)] for _ in range(CYCLES)]  
    number_of_collisions = [0]*CYCLES
    time_at_intersection = [[[] for _ in range(NUMBER_OF_ROVERS)] for _ in range(CYCLES)]
    throughput = [0]*CYCLES


    script_directory = os.path.dirname(os.path.abspath(__file__))
    #clear the CSV files:
    for i in range(1,NUMBER_OF_ROVERS+1):
        filename = os.path.join(script_directory,'CSV',f'APPROVAL{i}.csv')
        with open(filename, mode='w', newline='', encoding='utf-8') as file: 
            pass
        filename = os.path.join(script_directory,'CSV',f'KPS{i}.csv')
        with open(filename, mode='w', newline='', encoding='utf-8') as file: 
            pass



    for sim_cycle in range(CYCLES):
        # rover startpositions
        Cycle_Seed = SEED + sim_cycle
        Starting_Direction = generate_random_start_directions(NUMBER_OF_ROVERS, Cycle_Seed)
        print(Starting_Direction)
        #Starting_Direction = DIRECTION
        START_POSITIONS = generate_random_start_position(NUMBER_OF_ROVERS,Starting_Direction, ATTENTION_POINT_MIN, ATTENTION_POINT_MAX, PATHPOSITION1, PATHPOSITION2, Cycle_Seed)
        #as long as no suitable starting positions are found: 
        a = 1
        while START_POSITIONS == 'Error':
            Seed_new = Cycle_Seed + a
            Starting_Direction = generate_random_start_directions(NUMBER_OF_ROVERS, Seed_new)
            print(Starting_Direction)
            START_POSITIONS = generate_random_start_position(NUMBER_OF_ROVERS,Starting_Direction, ATTENTION_POINT_MIN, ATTENTION_POINT_MAX, PATHPOSITION1, PATHPOSITION2, Seed_new)
            a += 1
        Random_Route = generate_random_route(NUMBER_OF_ROVERS, Starting_Direction, Cycle_Seed)
        #print(Random_Number)
        #Random_Number = ROUTES
        move_points = generate_routes(NUMBER_OF_ROVERS, Starting_Direction ,POSSIBLE_ROUTES, Random_Route)
        #print(move_points)
        TARGET_COORDINATES = generate_target(NUMBER_OF_ROVERS,Starting_Direction, POSSIBLE_TARGETS, Random_Route)
        #print(TARGET_COORDINATES)
        initial_priorities = generate_init_priorities(NUMBER_OF_ROVERS, MAX_PRIORITY ,Cycle_Seed)
        target_times = generate_target_times(NUMBER_OF_ROVERS,START_POSITIONS,Random_Route, WAITING_POINT_ROUTE, KP_DIST)

        for i in range(NUMBER_OF_ROVERS):
            priorities[sim_cycle][i] = initial_priorities[i+1]
        
        start_time = pygame.time.get_ticks()
      
                    
        # initialising count variables
        simulation_time = 0                           
        number_of_rovers_in_target = 0
        useful_comms = 0
        not_useful_comms = 0
        collisions_set = set()
        positions ={} # Positions of every rover
        rovers_to_remove_ids = set()
        
        # generating rover instances:
        rovers = [Rover(ID, START_POSITIONS[ID], TARGET_COORDINATES[ID], WIDTH, HEIGHT, COMM_TYPE, simulation_time, move_points[ID], RADIUS[ID], Random_Route[ID], WAITING_POINT_ROUTE[Random_Route[ID]],KP_ROUTE[Random_Route[ID]],initial_priorities[ID],target_times[ID],Buffer[ID], sim_cycle) for ID in range(1, NUMBER_OF_ROVERS + 1)]

        # Main loop
        running = True
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
            screen.fill(WHITE)
            
            # display the simulated time
            sim_milliseconds = int((simulation_time*1000)%1000)
            sim_seconds = int(((simulation_time*1000)//1000)%60)
            sim_minutes = int(simulation_time//60)
            time_text = f"{sim_minutes}:{sim_seconds:02}.{sim_milliseconds:03}"
            time_text_surface = font.render(time_text,True,BLACK)
            screen.blit(time_text_surface,(750,780))

            # display number of Cycles
            cyles_text = f"Cylce: {sim_cycle+1} / {CYCLES}"
            cycle_text_surface = font.render(cyles_text, True, BLACK)
            screen.blit(cycle_text_surface,(5,5))





            # Draw the target and obstacles 
            for name, target_coordinate in TARGET_COORDINATES.items():
                pygame.draw.rect(screen, BLACK, (target_coordinate[0] - 6, target_coordinate[1] - 6, 20, 20))
                text_surface = font.render("G", True, WHITE)
                screen.blit(text_surface, (target_coordinate[0] -3, target_coordinate[1] -3))

            # Draw AP and WP Points
            #WPs
            pygame.draw.line(screen, RED, (380,WAITING_POINT_MIN ), (420, WAITING_POINT_MIN), 2)
            pygame.draw.line(screen, RED, (380, WAITING_POINT_MAX), (420, WAITING_POINT_MAX), 2)
            pygame.draw.line(screen, RED, (WAITING_POINT_MIN, 380), (WAITING_POINT_MIN, 420), 2)
            pygame.draw.line(screen, RED, (WAITING_POINT_MAX, 380), (WAITING_POINT_MAX, 420), 2)
            #APs
            pygame.draw.line(screen, ORANGE, (380,ATTENTION_POINT_MIN), (420, ATTENTION_POINT_MIN), 2)
            pygame.draw.line(screen, ORANGE, (380, ATTENTION_POINT_MAX), (420, ATTENTION_POINT_MAX), 2)
            pygame.draw.line(screen, ORANGE, (ATTENTION_POINT_MIN, 380), (ATTENTION_POINT_MIN, 420), 2)
            pygame.draw.line(screen, ORANGE, (ATTENTION_POINT_MAX, 380), (ATTENTION_POINT_MAX, 420), 2)

            for obstacle in OBSTACLES:
                pygame.draw.polygon(screen, BLACK, obstacle)
        
            # move and draw the rovers
            
            for rover in rovers:
                positions[rover.id] = [rover.x,rover.y]
                rover.move(NUMBER_OF_ROVERS, positions,simulation_time, script_directory)
                rover.sim_time = simulation_time
                draw_rover(rover, screen)
                draw_path(rover, screen)                

                #Check if there is a collision
                for rover1 in rovers:
                    if rover != rover1:
                        if ((rover.x-rover1.x)**2+(rover.y-rover1.y)**2)**0.5 < (rover.radius + rover1.radius):
                            if not rover1.reached_target and not rover.reached_target:  # if one rover has already reached the target, the collision does not count
                                collision_key = tuple(sorted((rover.id, rover1.id)))
                                # Check if collision is already counted
                                if collision_key not in collisions_set:
                                    print(f"Kollision zwischen Fahrzeug {rover.id} und Fahrzeug {rover1.id}.")
                                    number_of_collisions[sim_cycle] += 1
                                    collisions_set.add(collision_key)

                # Check if rover has reached the target
                if rover.reached_target and not rover.counted:
                    number_of_rovers_in_target += 1
                    rover.counted = True
                    rover.elapsed_time_to_target = simulation_time
                    print(f"Elapsed simulated time until Rover {rover.id} has reached the target: {rover.elapsed_time_to_target:.1f}s (target time was {rover.target_time}s)")
                    positions[rover.id] = [0,0]
                    rover_times_delta [sim_cycle][rover.id-1] = round(rover.delay,2)
                    time_at_intersection[sim_cycle][rover.id-1] = round(rover.time_intersection_left-rover.time_at_attentionpoint,2)
                    rovers_to_remove_ids.add(rover.id)
                                      
                    # Check if all rovers have reached the target
                    if number_of_rovers_in_target >= NUMBER_OF_ROVERS:
                        for rover in rovers:
                            useful_comms += rover.useful_comms
                            not_useful_comms += rover.not_useful_comms
                        # read number of comunication messages from CSV file
                        filename = os.path.join(script_directory,'CSV',f'number_of_messages.csv')
                        with open(filename, mode='r', newline='', encoding='utf-8') as file:
                            reader = csv.reader(file)
                            for row in reader:
                                read_data = row[0]
                        if read_data:
                            if sim_cycle <1:
                                number_of_communication[sim_cycle] = float(read_data)
                            elif sim_cycle >= 1:
                                number_of_communication[sim_cycle] = float(read_data) - total_number_of_communication
                            total_number_of_communication = float(read_data)
                        # calculate intersection throughput
                        latest_intersection_leave = 0
                        first_intersection_entry = 999
                        for rover in rovers:
                            latest_intersection_leave = max(latest_intersection_leave, rover.time_intersection_left)
                            first_intersection_entry = min(first_intersection_entry, rover.time_at_attentionpoint)
                            throughput[sim_cycle] = round((NUMBER_OF_ROVERS/(latest_intersection_leave-first_intersection_entry))*60,2)     #number of vehicles per minute
                        print(f"min: {first_intersection_entry}")
                        print(f"max: {latest_intersection_leave}")
                        print("All Rovers have reached the target!")
                        elapsed_time = (pygame.time.get_ticks() - start_time) / 1000  
                        print(f"Elapsed real time: {elapsed_time:.1f}s")
                        print(f"Number of sent messages: {number_of_communication[sim_cycle]}")
                        print(f"Number of collisions: {number_of_collisions[sim_cycle]}")
                        print(f"Rover Times Delta: {rover_times_delta[sim_cycle]}")
                        print(f"Throughput: {throughput[sim_cycle]} Vehicles per minute")
                        running = False          
            simulation_time += TIME_STEP                            
                                     
            pygame.display.flip()
            clock.tick(50)
            rovers = [rover for rover in rovers if rover.id not in rovers_to_remove_ids]
            

       
    #saving data to csv files after all simulation cycles        
    write_rover_times_to_file(NUMBER_OF_ROVERS, number_of_collisions, number_of_communication, CYCLES, rover_times_delta, priorities,time_at_intersection,throughput, 'rover_times_data.csv')

    
    pygame.quit()
 
if __name__ == "__main__":
    main()
