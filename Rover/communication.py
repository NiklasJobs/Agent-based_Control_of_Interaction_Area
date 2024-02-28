def communication_to_other_rovers():
        # Logik zur Auswahl der Rover, die informiert werden sollen
        rovers_to_notify = full_communication()
        #rovers_to_notify = utility_per_robot()
        #rovers_to_notify = OCBC()
        #rovers_to_notify = random_communication()
        #rovers_to_notify = no_communication()

        for rover_id in rovers_to_notify:
            # Annahme: Hier wird die Nachricht an den Rover mit der ID rover_id gesendet
            send_message_to_rover(rover_id, "Neues Hindernis wurde entdeckt!")

def full_communication():
        selected_rover = [1, 2, 3, 4, 5] 
        return [selected_rover]
    
def send_message_to_rover(rover_id, message):
    # Hier wird die Nachricht an den ausgewählten Rover gesendet
    pass