# Master's thesis Niklas Jobs

The simulation program presented here is used to simulate two agent-based control systems for an interaction area in an AGV system. An interaction area is an area in which collisions between different vehicles can potentially occur. In the simulation program, an intersection with 4 entrances and exits is simulated as an example for an interaction area.

The first implemented control system is a hierarchical control system in which there is a manager agent and several vehicle agents. The manager agent is responsible for collision detection and prevention. The vehicles must provide the manager agent with information and are dependent on the manager agent's clearance in order to pass the intersection. 

The second implemented control system is a heterarchical control system consisting only of vehicle agents of equal rank. Collision detection and avoidance is carried out in parallel by all vehicle agents. Therefore, the vehicle agents exchange information directly with each other.

## Installation and Execution

Follow these steps to download and run the project:

_1. Download_

Clone the repository to your local machine:

bash
git clone https://github.com/Niklas-22/Control_of_Interaction_Area.git


*2. Adjust Parameters*

Modify the parameters in the `constants.py` file as needed.
(For example, the following parameters can be adjusted: number of vehicles, number of simulation cycles, size of the vehicles, speed of the vehicles, priority of vehicles)

*3. Start the Simulation*

Navigate to the project directory in your terminal and start the simulation using one of the following commands:

For heterarchical communication:

bash
Pade start-runtime PADE_Communication_heterarchisch.py


For hierarchical communication:

bash
Pade start-runtime PADE_Communication_hierarchisch.py


*4. Authentication*

When prompted for a username and password, simply press the `Enter` key.

*5. Evaluation*

After all simulation cycles have been executed, certain KPIs are saved in the rover_times_data.csv file (under output_logger -> plotted_data). These can be used to evaluate the simulation experiments. 

## Requirements

Ensure that the following requirements are met:

- PADE must be installed. (version 2.2.5) You can install PADE using `pip install pade`.
- pygame must be installed. (version 2.6.0)
- shapely must be installed. (version 2.0.6)
- twisted must be installed. (version 19.7.0)
- Works with Python 3.9.19

## Sources

Original simulation program:
https://github.com/hsu-aut/RIVA_Decentralized_Communication

PADE:
https://github.com/grei-ufc/pade
