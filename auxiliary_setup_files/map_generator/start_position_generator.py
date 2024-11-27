import os
from obstacles import OBSTACLES
from simulation.world.functions import generate_random_start_positions
from simulation.constants import NUMBER_OF_ROVERS, WIDTH, HEIGHT, INITIAL_DISTANCE

def write_output(output_filename):
    # generate the list with start positions
    start_positions = generate_random_start_positions(NUMBER_OF_ROVERS, OBSTACLES, WIDTH, HEIGHT, INITIAL_DISTANCE)
    
    # Ermittle den vollständigen Dateipfad zur Ausgabedatei
    full_output_path = os.path.join(os.path.dirname(__file__), output_filename)

    with open(full_output_path, 'w') as file:
        file.write("START_POSITIONS = {\n")
        for i, pos in enumerate(start_positions, 1):
            file.write(f"    {i}: {pos},\n")
        file.write("}\n")

# Hauptprogramm
if __name__ == "__main__":
    output_filename = 'start_positions.py'  # Passe den Ausgabedateinamen an

    write_output(output_filename)