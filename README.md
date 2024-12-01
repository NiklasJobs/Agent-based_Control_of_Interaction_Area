# Master's thesis Niklas Jobs

The simulation program presented here is used to simulate two agent-based control systems for an interaction area in an AGV system. An interaction area is an area in which collisions between different vehicles can potentially occur. In the simulation program, an intersection with 4 directions is simulated as an example for an interaction area.

The first implemented control system is a hierarchical control system in which there is a manager agent and several vehicle agents. The manager agent is responsible for collision detection and prevention. The vehicles must provide the manager agent with information and are dependent on the manager agent's approval in order to pass the intersection. 

The second implemented control system is a heterarchical control system consisting only of vehicle agents of equal rank. Collision detection and avoidance is carried out in parallel by all vehicle agents. Therefore, the vehicle agents exchange information directly with each other.

## Installation and Execution

Follow these steps to download and run the project:

**1. Download**

Clone the repository to your local machine:

bash
git clone https://github.com/NiklasJobs/Agent-based_Control_of_Interaction_Area


**2. Adjust Parameters**

Modify the parameters in the `constants.py` file as needed.
(For example, the following parameters can be adjusted: number of vehicles, number of simulation cycles, size of the vehicles, speed of the vehicles, priority of vehicles)

The files contained in folder `auxiliary_setup_files` can be used to make extensive changes to the simulated environment and the used waypoints. 
The `map_generator.py`can be used in combination withe the `map.xlsx` file to change the placement of obstacles and thus the layout of the simulated environment.
If necessary, the positions and distances of the waypoints can be calculated with file `Calculation_Waypoints.xlsx` and can then be inserted into the `constants.py` file.

**3. Start the Simulation**

Navigate to the project directory in your terminal and activate the virtual environment if necessary. Then start the simulation using one of the following commands:

For heterarchical communication:

bash
Pade start-runtime PADE_Communication_heterarchisch.py


For hierarchical communication:

bash
Pade start-runtime PADE_Communication_hierarchisch.py


**4. Authentication**

When prompted for a username and password, simply press the `Enter` key.

**5. Stop the Simulation**

To exit the simulation program, press `CTRL+C`.

**6. Evaluation**

After all simulation cycles have been executed, certain KPIs are saved in the `rover_times_data.csv` file (in folder output_logger/plotted_data). These can be used to evaluate the simulation experiments. 

## Requirements

For error-free execution of the program, PADE and all other required modules should be installed in a virtual environment.

Ensure that the following requirements are met:

- Works with Python 3.9.19 --> If Python 3.9.19 does not work for you, try Python 3.7 
- PADE must be installed. (version 2.2.5) You can install PADE using `pip install pade`. (if you have problems with the installation, make sure you are using Python 3.7 and install wheel using `pip install wheel`)
- pygame must be installed. (version 2.6.0)
- shapely must be installed. (version 2.0.6)
- twisted must be installed. (version 19.7.0 / 20.3.0)
- numpy must be installed (version 2.0.2)


## Sources

Original simulation program:
https://github.com/hsu-aut/RIVA_Decentralized_Communication

PADE:
https://github.com/grei-ufc/pade
