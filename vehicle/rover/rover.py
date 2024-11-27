from world.obstacles import OBSTACLES
import random
from shapely import Polygon, Point
from vehicle.communication import *
from vehicle.navigation import *
from vehicle.world_model import *
from constants import *
import csv
import os
 
 
MIN_DISTANCE = 10                  # minimal distance for an avoiding WP around an obstacle for Rovers
MAX_SPEED = 0.2                         # move speed: 0.1 == 1 m/s == 3,6km/h
MAX_MEMORY_COMM_CANDIDATES = 3      # memory limit for comm_candidates
 
 
# definition of class Rover
class Rover:
    instantiated_rovers = {}
    
    def __init__(self, id, start_position, target_coordinates, WIDTH, HEIGHT, COMM_TYPE, sim_time, move_points, RADIUS, route, WAITING_POINT, KP_ROUTE, init_priority,target_time, buffer, simulation_cycle):
        self.id = id
        self.start_position = start_position
        self.x, self.y = start_position
        self.target_coordinates = target_coordinates                                            # mission definition: reaching the target
        self.WIDTH = WIDTH
        self.HEIGHT = HEIGHT
        self.comm_type = COMM_TYPE
        self.radius = RADIUS
        self.saftey_radius = self.radius + 2
        self.saftey_buffer = buffer                                                         # saftey Radius od Rover                                                                            # radius for drawing
        self.color = (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255))   # colour for drawing
        self.world_model = []                                                                  # internal world model of the rover including obstacles
        self.avoiding_WP = []                                                                   # possible WP to avoid obstacles                                              # list of WP that will be used to reach target (initialized with target_coordinat)
        self.move_points = move_points
        self.comm_candidates = []                                                               # list of candidates that could be communicated when possible
        self.comm_active = True                                                                 # parameter to indicate, if communication is possible at the requested time (only relevant for some communication paradigms)
        Rover.instantiated_rovers[self.id] = self                                               # team registration
        self.counted = False                                                                    # relevant for counting the rovers, which reached the target
        self.reached_target = False                                                             # relevant for counting the rovers, which reached the target
        self.useful_comms = 0                                                                   # relevant for counting usefull communications
        self.not_useful_comms = 0                                                               # relevant for counting not usefull communications
        self.moved_distance = 0                                                                 # distance moved by the rover in meter [m]
        self.sim_time = sim_time                                                                # internal simulation time
        self.elapsed_time_to_target = 0                                                         # elapsed time until target reached [s]
        self.active_communications = 0                                                          # number of communications currently in the network
        self.number_of_known_obstacles = 0                                                      # relevant for evaluation of Environmental Awareness Ratio
        self.distance_to_target = distance_to_target(self.move_points, self.x, self.y, self.target_coordinates)          # distance to target following all move_points
        self.speed = MAX_SPEED
        self.max_speed = MAX_SPEED
        self.reached_waitingpoint = False
        self.reached_attentionpoint = False
        self.time_at_attentionpoint = None
        self.approval_waiting_point = None
        self.left_intersection = False
        self.route = route
        self.WAITING_POINT = WAITING_POINT
        self.KP_Route = KP_ROUTE
        self.priority = init_priority
        self.target_time = target_time
        self.time_at_waitingpoint = 0
        self.speed_regulator = True # controls wether the speed of a rover can be adjusted or not
        self.delay = 0
        self.time_intersection_left = 0
        self.simulation_cycle = simulation_cycle #used to distinguish between different intersection scenarios


    def move(self,number_of_rover,rover_positions, simulation_time, script_directory):
        if not self.reached_target:
            if self.move_points:
                current_target_x, current_target_y = self.move_points[0]
            # handling of unlikely error event
            else:
                self.move_points = perform_navigation(self.world_model, self.x, self.y, self.target_coordinates, self.WIDTH, self.HEIGHT, MIN_DISTANCE, self.move_points)
                current_target_x, current_target_y = self.move_points[0]
            #heading to target
            dx = current_target_x - self.x
            dy = current_target_y - self.y
            #distance to target
            distance_next_WP = ((dx ** 2) + (dy ** 2)) ** 0.5
 
            # observation of the world until target is reached
            if self.number_of_known_obstacles < NUMBER_OF_OBSTACLES:         # no obstacle detection needed, when all obstacles are known
                self.obstacle_detection()

            #self.communication()   # not needed
 
            #WP management
            if distance_next_WP <= 0.4:  
                self.x = current_target_x
                self.y = current_target_y
                self.move_points.pop(0)
                if not self.move_points and self.distance_to_target <= 1:
                    self.reached_target = True
                else:
                    self.move_points = perform_navigation(self.world_model, self.x, self.y, self.target_coordinates, self.WIDTH, self.HEIGHT, MIN_DISTANCE, self.move_points)                     # setting back to target, in case of error
            else:
                # moving the rover
                self.new_x = self.x + dx / distance_next_WP * self.speed
                self.new_y = self.y + dy / distance_next_WP * self.speed

                #rover is only allowed to move, if no other rover is in his way    
                conflict = 0
                for i in range(1,number_of_rover+1):
                        if i in rover_positions:
                            if i != self.id:
                                #if (abs(self.new_x - rover_positions[i][0]) < self.saftey_radius+5) and (abs(self.new_y - rover_positions[i][1]) < self.saftey_radius+5):
                                    if ((self.x-rover_positions[i][0])**2+(self.y-rover_positions[i][1])**2)**0.5 < self.saftey_radius+5:
                                        if (abs(self.new_x - rover_positions[i][0]) + (abs(self.new_y - rover_positions[i][1]))) < (abs(self.x - rover_positions[i][0]) + (abs(self.y - rover_positions[i][1]))):
                                            conflict = 1

                #rover is only allowed to move across the WP if approval is given and the rover is in the approved time slot
                if self.new_x + self.radius > WAITING_POINT_MIN and self.new_y + self.radius > WAITING_POINT_MIN and self.new_x - self.radius < WAITING_POINT_MAX and self.new_y - self.radius < WAITING_POINT_MAX:
                        if not self.reached_waitingpoint:
                            if (self.approval_waiting_point) != 1 or (simulation_time < self.time_at_waitingpoint):
                                conflict = 1
                            elif ((self.approval_waiting_point == 1) and (simulation_time > self.time_at_waitingpoint +0.1)):
                                conflict = 1
                                self.approval_waiting_point = 0
                                print('Sim Time > Time at WP')
                                self.calculation(KP_DIST, simulation_time,script_directory)
                                
                # rover is only allowed to move if there is no conflict
                if conflict == 0:
                    self.x += dx / distance_next_WP * self.speed
                    self.y += dy / distance_next_WP * self.speed
                    self.moved_distance += ((((dx / distance_next_WP * self.speed) ** 2) + ((dy / distance_next_WP * self.speed) ** 2)) ** 0.5)/10    # /10 as 10units in the world = 1m
                    self.distance_to_target = distance_to_target(self.move_points, self.x, self.y, self.target_coordinates)
                    
                 #Check if Wainting Point is reached:
                if self.x + self.radius > WAITING_POINT_MIN and self.y + self.radius > WAITING_POINT_MIN and self.x - self.radius < WAITING_POINT_MAX and self.y - self.radius < WAITING_POINT_MAX:
                    if not self.reached_waitingpoint:
                        self.reached_waitingpoint = True
                        print(f"Rover {self.id} has passed the Waiting Point.({simulation_time:.3f}s)")
                        self.speed = self.max_speed #  if WP is passed then speed is set to max speed
                        send_data = {}
                        send_data['INo'] = self.simulation_cycle +1
                        send_data['P'] = self.priority
                        send_data['WP'] = self.reached_waitingpoint
                        send_data['B'] = self.saftey_buffer
                        for kp in self.KP_Route:
                            time = (float(KP_DIST[self.route-1,kp-1]/(self.max_speed*100)+self.time_at_waitingpoint))
                            send_data[kp]=time

                        filename = os.path.join(script_directory,'CSV',f'KPS{self.id}.csv')
                        with open(filename, mode='w', newline='', encoding='utf-8') as file:
                            keys = list(send_data.keys())
                            writer = csv.DictWriter(file,fieldnames= keys)
                            writer.writeheader()
                            writer.writerow(send_data)
                
                #Check if Attention Point is reached & check if there is an approval/decline:
                if self.x >= ATTENTION_POINT_MIN and self.y >= ATTENTION_POINT_MIN and self.x <= ATTENTION_POINT_MAX and self.y <= ATTENTION_POINT_MAX:
                    if not self.reached_attentionpoint:
                        self.reached_attentionpoint = True
                        self.time_at_attentionpoint = simulation_time
                        self.calculation(KP_DIST, simulation_time,script_directory)
                        print(f"Rover {self.id} has reached the Attention Point.({simulation_time:.3f}s)"),
                    elif self.reached_attentionpoint and not self.left_intersection:

                        self.check_approval(script_directory)
                        if self.approval_waiting_point == 0 or (self.time_at_waitingpoint <= simulation_time and not self.reached_waitingpoint):
                            self.speed_regulator = True # speed can be adjusted
                            self.approval_waiting_point = 0
                            self.calculation(KP_DIST, simulation_time,script_directory)
                        elif self.approval_waiting_point == 1 and not self.reached_waitingpoint and self.speed_regulator:
                            self.distance_to_waitingpoint = abs((self.x- self.WAITING_POINT[0]) + (self.y- self.WAITING_POINT[1]))
                            if self.distance_to_waitingpoint > 10:  #speed should not be further reduced, if rover is already nerby the WP, because then it cannot be avoided to stop the rover
                                self.speed = max(min((self.distance_to_waitingpoint/(self.time_at_waitingpoint-simulation_time)/100),self.max_speed), 0.05) # idealy the rover should not stop, but the rover must be within the min and max speed
                                self.speed_regulator = False    # Prohibits permanent speed adjustment
                            else:
                                self.speed = self.max_speed
                        elif self.approval_waiting_point == 2:
                            self.speed = self.max_speed
                            self.speed_regulator = True
                            self.time_at_waitingpoint = 0
                            self.calculation(KP_DIST,simulation_time,script_directory)
                
                #Check if intersection has been left:
                if self.x + self.radius < WAITING_POINT_MIN or self.y + self.radius < WAITING_POINT_MIN or self.x - self.radius > WAITING_POINT_MAX or self.y - self.radius > WAITING_POINT_MAX:
                    if self.reached_waitingpoint == True:
                        if self.left_intersection == False:
                            print(f'Rover{self.id} has left the intersection.')
                            self.left_intersection = True
                            self.time_intersection_left = simulation_time
                            send_data = {}
                            send_data['INo'] = self.simulation_cycle +1 # INo = intersection number
                            send_data['IL'] = self.left_intersection
                            filename = os.path.join(script_directory,'CSV',f'KPS{self.id}.csv')
                            with open(filename, mode='w', newline='', encoding='utf-8') as file:
                                keys = list(send_data.keys())
                                writer = csv.DictWriter(file,fieldnames= keys)
                                writer.writeheader()
                                writer.writerow(send_data)
                            filename2 = os.path.join(script_directory,'CSV',f'APPROVAL{self.id}.csv')
                            with open(filename2, mode='w', newline='', encoding='utf-8') as file:
                                pass

                
                   
                    


                

    def obstacle_detection(self):
        detection_range = Point(self.x, self.y).buffer(WIDTH)          #sensor range
 
        #check if obstacles are in sensor range
        for obstacle in OBSTACLES:
            if detection_range.intersects(Polygon(obstacle)) and not obstacle_in_world_model(obstacle, self.world_model): #detection of unknown obstacle
                
                # update of world model
                self.number_of_known_obstacles += 1
                if len(self.comm_candidates) >= MAX_MEMORY_COMM_CANDIDATES:         # ensuring max length of comm_candidate
                    self.comm_candidates.pop(0)                                     # removal of first element (FIFO)
                self.comm_candidates.append(obstacle)
                world_model_update(obstacle, self.world_model)
                
                # update of navigation if detected obstacle intersects with current path
                if worldmodel_intersects_path(self.world_model, self.move_points, self.x, self.y, MIN_DISTANCE):
                    self.move_points = perform_navigation(self.world_model, self.x, self.y, self.target_coordinates, self.WIDTH, self.HEIGHT, MIN_DISTANCE, self.move_points)                                                    
                
    def communication(self):        # NOT USED!!!    
        # perform communication
        if self.comm_type == "No_Comm":
            no_comm()
        elif self.comm_type == "Timing_Selective":
            if self.comm_active and self.comm_candidates:
                self.comm_active = False
                timing_selective_comm(self, self.comm_candidates, self.instantiated_rovers)
        elif self.comm_type == "Content_Selective":
            if self.comm_candidates:
                content_selective_comm(self, self.comm_candidates, self.instantiated_rovers)
        elif self.comm_type == "Receiver_Selective":
            if self.comm_candidates:
                receiver_selective_comm(self, self.comm_candidates, self.instantiated_rovers)
        elif self.comm_type == "Integrated":
            if self.comm_active and self.comm_candidates:
                self.comm_active = False
                integrated_comm(self, self.comm_candidates, self.instantiated_rovers)
        else:
            if self.comm_candidates:
                full_comm(self, self.comm_candidates, self.instantiated_rovers)
                                         
                
    def receive_message(self, obstacle, sender):    # NOT USED!!!
        if not self.reached_target:                                 # only receive data if target not reached
            if obstacle_in_world_model(obstacle, self.world_model): #reception of already known obstacle
                self.not_useful_comms += 1
            else:                                                   #reception of unknown obstacle
                # update of world model
                self.number_of_known_obstacles += 1
                world_model_update(obstacle, self.world_model)
 
                # update of navigation if percepted obstacle intersects with current path
                if worldmodel_intersects_path(self.world_model, self.move_points, self.x, self.y, MIN_DISTANCE):   
                    self.move_points = perform_navigation(self.world_model, self.x, self.y, self.target_coordinates, self.WIDTH, self.HEIGHT, MIN_DISTANCE, self.move_points)
                    self.useful_comms += 1
                else:  # not usefull communication
                    self.not_useful_comms += 1
        else:
            self.not_useful_comms += 1  

    def check_approval(self,script_directory):
        filename = os.path.join(script_directory,'CSV',f'APPROVAL{self.id}.csv')
        approval = []

        if os.path.isfile(filename):
            with open(filename, mode='r', newline='', encoding='utf-8') as file:
                reader = csv.reader(file)
                for row in reader:
                    if row:
                        number = int(float(row[0]))
                        approval.append((number))
                #print(f'APPROVAL = {approval}')
            if approval:
                if approval[0] == 1:    # Approval for requested timeslots
                    self.approval_waiting_point = 1
                    if self.reached_waitingpoint:
                        self.speed = MAX_SPEED
                elif approval[0] == 0:  # Rejection for requested timeslots
                    self.approval_waiting_point = 0
                elif approval[0] == 2:  # Request for earlier timeslots
                    self.approval_waiting_point = 2

            else:
                self.approval_waiting_point = None

    def calculation(self,KP_DIST,simulation_time,script_directory):
        filename = os.path.join(script_directory,'CSV',f'KPS{self.id}.csv')
        filename2 = os.path.join(script_directory,'CSV',f'APPROVAL{self.id}.csv')
        time_step = TIME_ADJUSTMENT


        if self.time_at_waitingpoint <= 0:
            #self.distance_to_waitingpoint = abs((self.start_position[0]- self.WAITING_POINT[0]) + (self.start_position[1]- self.WAITING_POINT[1]))
            #self.time_at_waitingpoint = self.distance_to_waitingpoint / (self.speed*100)
            self.distance_to_waitingpoint = abs((self.x - self.WAITING_POINT[0])) + abs((self.y - self.WAITING_POINT[1])) - self.radius
            self.time_at_waitingpoint = (self.distance_to_waitingpoint / (self.speed*100)) + simulation_time

            send_data = {}
            send_data['INo'] = self.simulation_cycle +1
            send_data['P'] = self.priority
            send_data['WP'] = self.reached_waitingpoint
            send_data['B'] = self.saftey_buffer
            send_data['tAP'] = self.time_at_attentionpoint
            send_data['tWP'] = self.time_at_waitingpoint
            for kp in self.KP_Route:
                time = (float((KP_DIST[self.route-1,kp-1]+self.radius)/(self.max_speed*100)+self.time_at_waitingpoint))
                send_data[kp]=time

            with open(filename, mode='w', newline='', encoding='utf-8') as file:
                keys = list(send_data.keys())
                writer = csv.DictWriter(file,fieldnames= keys)
                writer.writeheader()
                writer.writerow(send_data)
            with open(filename2, mode='w', newline='', encoding='utf-8') as file:
                pass
            self.approval_waiting_point = None

        if self.approval_waiting_point == 0:
            self.distance_to_waitingpoint = abs((self.x- self.WAITING_POINT[0]) + (self.y- self.WAITING_POINT[1]))
            if self.time_at_waitingpoint <= simulation_time:
                self.time_at_waitingpoint = simulation_time + time_step
            else: 
                self.time_at_waitingpoint += time_step

            self.delay += time_step

            #self.speed = min(self.distance_to_waitingpoint/((self.time_at_waitingpoint-simulation_time)*100),self.max_speed)
            print(f'Rover{self.id}: t_WP={self.time_at_waitingpoint}')
            print(f'Rover{self.id}: v={self.speed}')

            send_data = {}
            send_data['INo'] = self.simulation_cycle +1
            send_data['P'] = self.priority
            send_data['WP'] = self.reached_waitingpoint
            send_data['B'] = self.saftey_buffer
            send_data['tWP'] = self.time_at_waitingpoint
            for kp in self.KP_Route:
                time = (float((KP_DIST[self.route-1,kp-1]+self.radius)/(self.max_speed*100)+self.time_at_waitingpoint))
                send_data[kp]=time

            with open(filename, mode='w', newline='', encoding='utf-8') as file:
                keys = list(send_data.keys())
                writer = csv.DictWriter(file,fieldnames= keys)
                writer.writeheader()
                writer.writerow(send_data)
            with open(filename2, mode='w', newline='', encoding='utf-8') as file:
                pass
            self.approval_waiting_point = None         