
def no_communication():
    pass

def full_communication(obstacle, instantiated_rovers, sender):
    for rover_id, receiver in instantiated_rovers.items():
        if rover_id != sender:  # Avoid sending message to itself
            receiver.receive_message(obstacle, sender)

def three_receives(obstacle, instantiated_rovers, sender):
    for rover_id, receiver in instantiated_rovers.items():
        if rover_id == 3 and rover_id != sender:  # Avoid sending message to itself
            receiver.receive_message(obstacle, sender)