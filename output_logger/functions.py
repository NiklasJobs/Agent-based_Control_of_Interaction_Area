import os
import csv
import numpy as np

def write_rover_times_to_file(NUMBER_OF_ROVERS, number_of_collisions,number_of_communications, Cycles,rover_time_delta, priority, time_at_intersection,throughput,filename):
    communication_type = None
    script_directory = os.path.dirname(os.path.abspath(__file__))
    name = os.path.join(script_directory,'communication_type.csv')
    with open(name, mode='r', newline='', encoding='utf-8') as file:
        reader = csv.reader(file)
        for row in reader:
            if row:
                communication_type = row[0]

    output_folder = os.path.join(os.getcwd(), 'output_logger', 'plotted_data')
    output_path = os.path.join(output_folder, filename)
    with open(output_path, mode='w', newline='') as file:
        writer = csv.writer(file, delimiter=";")
        writer.writerow(["Communication Type:", communication_type])
        writer.writerow(["Number of Rovers: ", NUMBER_OF_ROVERS])
        writer.writerow([])
        #max_length = max(len(d) for d in data)
        writer.writerow([f'Cycle {i}' for i in range(1, Cycles+1) for j in range(3)])
        collisions_row = []
        for i in range(Cycles):
                collisions_row.extend(['Number of Collisions',number_of_collisions[i],''])
        writer.writerow(collisions_row)
        communication_row = []
        for i in range(Cycles):
                communication_row.extend(['Number of Communication',number_of_communications[i],''])
        writer.writerow(communication_row)
        throughput_row = []
        for i in range(Cycles):
                throughput_row.extend(['Throughput',throughput[i],''])
        writer.writerow(throughput_row)
        header_row = []
        for i in range(Cycles):
            header_row.extend(['priority','delay','time at intersection'])
        writer.writerow(header_row)
        for i in range(NUMBER_OF_ROVERS):
            data_row = []
            for sim_cycle in range(Cycles):
                data_row.extend([priority[sim_cycle][i],rover_time_delta[sim_cycle][i],time_at_intersection[sim_cycle][i]])
            writer.writerow(data_row)