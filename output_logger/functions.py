import os
import csv

def write_to_file(comm_type, number_of_rovers, data, filename):
    output_folder = os.path.join(os.getcwd(), 'simulation', 'output_logger')
    output_path = os.path.join(output_folder, filename)
    
    with open(output_path, mode='w', newline='') as file:
        writer = csv.writer(file, delimiter=";")
        writer.writerow([comm_type, "Number of Rovers: ", number_of_rovers])
        
        
        # Bestimme die maximale Anzahl von Testdurchläufen
        max_length = max(len(d) for d in data)
        
        # Schreibe die Kopfzeile (Testdurchläufe)
        writer.writerow([f'Trial {i+1}' for i in range(max_length)])
        
        # Schreibe die Zeiten der Rover in die CSV-Datei
        for item in data:
            writer.writerow(item)


def plot_network_load(simulation_time, network_load, network_load_line, rovers, active_communications):
    if round(simulation_time % 0.02, 10) == 0:
        if network_load_line == len(network_load):
            network_load.append([])
        for rover in rovers:
            active_communications += rover.active_communications
        network_load[network_load_line].append(active_communications)
        active_communications = 0
        network_load_line += 1
        
def plot_rover_times(rovers, rover_times, useful_comms, not_useful_comms):
    for rover in rovers:
        rover_times[rover.id - 1].append(round(rover.elapsed_time_to_target))
    rover_times[-2].append(useful_comms)
    rover_times[-1].append(not_useful_comms)