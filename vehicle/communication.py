import threading
import time
import random

def full_comm(obstacle, instantiated_rovers, sender):
    for rover_id, receiver in instantiated_rovers.items():
        if rover_id != sender and not receiver.reached_target:  # Avoid sending message to itself and to rovers that reached the target
            threading.Thread(target=delayed_receive_message, args=(receiver, obstacle, sender)).start()

def minimum_distance_comm(obstacle, instantiated_rovers, sender, x_coordinate, y_coordinate):
    min_distance = int('inf')                                       # initializing with infinite
    min_dist_vehicle = None

    for rover_id, receiver in instantiated_rovers.items():
        if rover_id != sender:                                      # Avoid sending message to itself
            distance = ((receiver.x - x_coordinate) ** 2 + (receiver.y - y_coordinate) ** 2) ** 0.5

            if distance < min_distance:
                min_distance = distance
                min_dist_vehicle = receiver

    if min_dist_vehicle is not None:                               # check if vehicle with min_distance was found
        min_dist_vehicle.receive_message(obstacle, sender)

def max_utility_comm(obstacle, instantiated_rovers, sender):
    max_team_utility = 0                                            # initializing with zero
    max_util_vehicle = None

    for rover_id, receiver in instantiated_rovers.items():
        if rover_id != sender:                                      # Avoid sending message to itself
            team_utility = receiver.estimated_team_utility

            if team_utility > max_team_utility:
                max_team_utility = team_utility
                max_util_vehicle = receiver

    if max_util_vehicle is not None:                               # check if vehicle with max_utility was found
        max_util_vehicle.receive_message(obstacle, sender)

def direct_impact_comm():
    pass

def future_impact_comm():
    pass

def no_comm():
    pass

def delayed_receive_message(receiver, obstacle, sender):
    sender.active_communications += 1               
    
    # Function to send message with delay
    delay_ms = random.randint(20, 100)/1000  # Generate random delay between 20 and 100 ms
    time.sleep(delay_ms)
    receiver.receive_message(obstacle, sender)
    
    sender.active_communications += 1