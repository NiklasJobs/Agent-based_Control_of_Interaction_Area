import random
import math
from shapely.geometry import Point, Polygon
from vehicle.navigation import avoiding_WP_generation
from constants import *

def generate_random_start_directions(num_positions, seed=None):
   directions= ["N", "O", "S", "W"]
   start_directions = []
   if seed is not None:
      random.seed(seed)

   while len(start_directions) < num_positions:
        selected_direction = random.choice(directions)
        start_directions.append(selected_direction)
           
   start_directions_dict = {i+1: pos for i, pos in enumerate(start_directions)}
   return start_directions_dict



def generate_random_start_position(num_positions, start_direction, ATTENTION_POINT_MIN, ATTENTION_POINT_MAX,position_k, position_g, seed = None):
   if seed is not None:
      random.seed(seed)
   start_positions = []
   j = 1
   error = 0  #Counts the runs for which no valid points were found
   new_attempts = 0  #Counts the complete new_attemps
   
   def valid_position(new_position):
    for position in start_positions:
        distance = math.sqrt((new_position[0] - position[0])**2 + (new_position[1] - position[1])**2)
        if distance < 20:
            return False
    return True
   
   while len(start_positions) < num_positions:
        if start_direction[j] == "N":
            x = position_k
            y = random.randint(0, ATTENTION_POINT_MIN)
            new_position = (x,y)
          #  start_positions.append((x,y))
          #  j +=1

        elif start_direction[j] == "O":
            x = random.randint(ATTENTION_POINT_MAX, WIDTH)
            y = position_k
            new_position = (x,y)
           # start_positions.append((x,y))
           # j +=1

        elif start_direction[j] == "S":
            x = position_g
            y = random.randint(ATTENTION_POINT_MAX, WIDTH)
            new_position = (x,y)
           # start_positions.append((x,y))
           # j +=1

        elif start_direction[j] == "W":
            x = random.randint(0, ATTENTION_POINT_MIN)
            y = position_g
            new_position = (x,y)
            #start_positions.append((x,y))
            #j +=1

        if valid_position (new_position):
            start_positions.append(new_position)
            j += 1
            error = 0
        else: 
           error += 1
        
        if error > 30:
           j = 1
           start_positions = []
           new_attempts += 1
           print("Error > 30 : Restart Position Generation")
        
        if new_attempts > 30:
           print ("New Start direction generation")
           return 'Error'
    
    
   start_positions_dict = {i+1: pos for i, pos in enumerate(start_positions)}
   return start_positions_dict

def generate_random_route (num_positions, start_direction, seed = None):
   if seed is not None:
      random.seed(seed)
   random_dict = {}
   for i in range(1, num_positions+1):
       if start_direction[i] == "N":
        random_dict[i] = random.randint(1,3)

       elif start_direction[i] == "O":
        random_dict[i] = random.randint(4,6)

       elif start_direction[i] == "S":
        random_dict[i] = random.randint(7,9)           
            
       elif start_direction[i] == "W":
        random_dict[i] = random.randint(10,12)
   
   return random_dict


def generate_routes (num_positions, start_direction, possible_routes, random_number):  
   routes_dict = {}
   for i in range(1, num_positions+1):
        r = random_number[i]
        routes_dict[i] = possible_routes[r][:]
   
   return routes_dict

def generate_target (num_positions, start_direction, possible_targets, random_number):
   target_dict = {}
   for i in range(1, num_positions+1):
       if start_direction[i] == "N":
        r = random_number[i]
        target_dict[i] = possible_targets[r]

       elif start_direction[i] == "O":
        r = random_number[i]
        target_dict[i] = possible_targets[r]

       elif start_direction[i] == "S":
        r = random_number[i]
        target_dict[i] = possible_targets[r]          
            
       elif start_direction[i] == "W":
        r = random_number[i]
        target_dict[i] = possible_targets[r]
   
   return target_dict


def generate_init_priorities(num_positions,max_priority ,seed = None):
    if seed is not None:
      random.seed(seed)
    priorities = []
    for i in range(1, num_positions+1):
         r = random.randint(1, max_priority)
         #r = 1
         priorities.append(r)

    prio_dict = {i+1: pos for i, pos in enumerate(priorities)}
    return prio_dict

def generate_target_times(num_positions, starting_points, routes, wp_route, KP_Dist):
   #1. calculate minimum time to target
   #2. generate random number bewteem min time to target and upper end (min time to target + M)
    target_times_dict = {}
    for i in range(1, num_positions+1):
        r = routes[i]
        dist_to_WP = abs(starting_points[i][0] - wp_route[r][0]) + abs(starting_points[i][1] - wp_route[r][1])
        dist_KP = KP_Dist[r-1]
        dist_to_last_KP = max(dist_KP)
        dist_to_target = dist_to_WP + dist_to_last_KP + (HEIGHT-PATHPOSITION2-TURNING_RADIUS)
        min_time_to_target = dist_to_target/(0.2*100)
        target_time = min_time_to_target                                            #min target time
        #target_time = random.uniform(min_time_to_target, min_time_to_target+10)    #Random Target Time
        target_times_dict[i] = round(float(target_time),2)
    return target_times_dict