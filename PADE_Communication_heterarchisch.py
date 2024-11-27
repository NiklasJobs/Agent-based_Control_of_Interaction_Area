from pade.misc.utility import display_message, start_loop, call_later
from pade.acl.messages import ACLMessage
from pade.core.agent import Agent
from pade.acl.aid import AID
from pade.behaviours.protocols import FipaRequestProtocol
from pade.behaviours.protocols import TimedBehaviour
from constants import *
import json
import csv
import numpy as np
import os
import subprocess
from sys import argv

script_directory = os.path.dirname(os.path.abspath(__file__))
number_of_rover = NUMBER_OF_ROVERS
number_of_messages = 0 # variable to count the number of sent messages

# clearing of CSV files
for i in range(1, number_of_rover+1):  
    file = os.path.join(script_directory,'CSV',f'KPS{i}.csv')
    with open(file, mode='w', newline='', encoding='utf-8') as file:
        pass

filename = os.path.join(script_directory,'CSV',f'number_of_messages.csv')    
with open(filename, mode='w', newline='', encoding='utf-8') as file:
    pass


# SimAgent only starts the simulation and sends the number of sent messages to the simulation script
class SimAgent(Agent):
    def __init__(self, aid):
        super(SimAgent, self).__init__(aid=aid, debug=False)
    
    def on_start(self):
        super(SimAgent, self).on_start()
        subprocess.Popen(['python', 'main.py']) # start simulation
        #write communication Type for data analysis
        filename = os.path.join(script_directory,'output_logger',f'communication_type.csv')    
        with open(filename, mode='w', newline='', encoding='utf-8') as file:
            writer = csv.writer(file)
            writer.writerow(['heterarchical'])
        count_messages_behavior = simulation_analysis(self,2.0)
        self.behaviours.append(count_messages_behavior)

    def count_messages(self):
        filename = os.path.join(script_directory,'CSV',f'number_of_messages.csv')    
        with open(filename, mode='w', newline='', encoding='utf-8') as file:
            writer = csv.writer(file)
            writer.writerow([number_of_messages])


class simulation_analysis(TimedBehaviour):
    def __init__(self,agent,time):
        super(simulation_analysis,self).__init__(agent,time)
        
    def on_time(self):
        super(simulation_analysis,self).on_time()
        self.agent.count_messages()


# Timed Behavior from Rover Agent
class CheckKPSBehavior(TimedBehaviour):
    def __init__(self,agent,time):
        super(CheckKPSBehavior,self).__init__(agent,time)
        
    def on_time(self):
        super(CheckKPSBehavior,self).on_time()
        self.agent.check_KPS()

##################################################################
class RoverAgent(Agent):
    def __init__(self, aid, ID, all_agents_aid):
        super(RoverAgent, self).__init__(aid=aid, debug=False)
        # like IM Agent
        self.ID = ID
        self.aid = aid
        self.all_agents_aid = all_agents_aid
        self.t = [[None for _ in range(12)] for _ in range(number_of_rover)]
        self.p = [0]*(number_of_rover)  # priorities of rover
        self.p_sent = 0     # own priority that is sent to other rover (important when own priority is changed)
        self.WP = [False]*number_of_rover #1 if rover i has passed the WP
        self.buffer = [None]*(number_of_rover) #saftey buffer for rover i
        self.G = np.ones((number_of_rover,12))  
        self.F_i =  np.ones(number_of_rover)     # own approval in relation to rover i
        self.F = 0          # own total approval
        self.F_other = [0]*(number_of_rover)    #virtual approval for other rover (important for deciding how to react to a message from another Rovers)
        self.number_of_rover = number_of_rover  
        self.sd = [None]*number_of_rover #start direction of Rover i
        self.time_at_AP = [None]*number_of_rover    # time when rover i passes the attention point
        self.time_at_WP = [None]*number_of_rover    # time when rover i plans to pass the waiting point
        self.excluded_agents = [] # list of rovers that have left the intersection and do not need to be informed
        self.position = [None]*number_of_rover  #position of rover i
        self.number_of_sent_messages = 0    #number of messages sent by rover i
        self.intersection_ID = 0    #ID of the upcoming intersection (in this simulation study the intersection ID equals the number of the simulation Cycle)


    def on_start(self):
        super(RoverAgent, self).on_start()
        display_message(self.aid.localname, "Rover Agent gestartet!")
        display_message(self.aid.localname,"{}".format(AID))
        check_kps_behavior = CheckKPSBehavior(self,1.0)
        self.behaviours.append(check_kps_behavior)


    def check_KPS(self):
        p_new = 0
        t_new = [None]*12
        WP_new = False
        IL_new = False
        filename = os.path.join(script_directory,'CSV',f'KPS{self.ID}.csv')
        read_data = []
        positive_elements = []
    
        with open(filename, mode='r', newline='', encoding='utf-8') as file:
            reader = csv.DictReader(file)
            for row in reader:
                read_data.append(row)

        if read_data:
            key = 'INo'
            if key in read_data[0]:
                self.intersection_ID = int(read_data[0][key])
            key = 'P'
            if key in read_data[0]:
                p_new = max(float(read_data[0][key]), self.p[self.ID-1])
            key = 'WP'
            if key in read_data[0]:
                WP_new = read_data[0][key]
            key = 'IL'
            if key in read_data[0]:
                IL_new = read_data[0][key]  
            key = 'B'
            if key in read_data[0]:
                self.buffer[self.ID-1] = float(read_data[0][key])
            key = 'tAP'
            if key in read_data[0]:
                self.time_at_AP[self.ID-1] = float(read_data[0][key])
            key = 'tWP'
            if key in read_data[0]:
                self.time_at_WP[self.ID-1] = float(read_data[0][key])
            for k in range(1,13):
                key = str(k)
                if key in read_data[0]:
                    t_new[k-1] = float(read_data[0][key])
                    positive_elements.append(t_new[k-1])

            if positive_elements and self.sd[self.ID-1] is None:
                first_kp = (t_new.index(min(positive_elements)))+1   # The start direction is determined based on the first KP used
                
                if first_kp == 4 or first_kp == 5:
                    self.sd[self.ID-1] = 'N'
                elif first_kp == 1 or first_kp == 6:
                    self.sd[self.ID-1] = 'O'
                elif first_kp == 2 or first_kp == 7:
                    self.sd[self.ID-1] = 'S'
                elif first_kp == 3 or first_kp == 8:
                    self.sd[self.ID-1] = 'W'
                number = 1
                for i in range(number_of_rover):
                    if (i != (self.ID-1)) and(self.sd[i] == self.sd[self.ID-1]):
                        if self.time_at_AP[i] < self.time_at_AP[self.ID-1]:
                            number = max(number, self.position[i]+1)
                self.position[self.ID-1] = number
                print(f'Rover{self.ID}: First KP = {first_kp}; SD: {self.sd[self.ID-1]}; position: {self.position[self.ID-1]}')                       

        # if there is new data, then update the data sent to all other Rover
        # if the rover has left the intersection, all other rovers should be informed and the intersection-specific information should be cleared
        if (IL_new == 'True'):
            self.send_data = {}
            self.send_data ['ID'] = self.ID
            self.send_data['INo'] = self.intersection_ID
            self.send_data['IL'] = IL_new
            self.send_broadcast_message(self.excluded_agents)
            # reset all data
            self.p = [0]*(number_of_rover)
            self.p_sent = 0
            self.t = [[None for _ in range(12)] for _ in range(number_of_rover)]
            self.WP = [False]*number_of_rover
            self.sd = [None]*number_of_rover
            self.position = [None]*number_of_rover
            self.time_at_AP = [None]*number_of_rover
            self.time_at_WP = [None]*number_of_rover
            self.G = np.ones((number_of_rover,12))  
            self.F_i =  np.ones(number_of_rover)
            self.excluded_agents = []
            self.intersection_ID += 1   # intersection ID is already set on the next intersection
            global number_of_messages
            number_of_messages = number_of_messages + self.number_of_sent_messages
            self.number_of_sent_messages = 0
            with open(filename, mode='w', newline='', encoding='utf-8') as file:
                pass
            print(f'Rover{self.ID} reset all data')
        else:
            # if there is new data, then update the data sent to all other Rover
            if (p_new != self.p_sent) or (t_new != self.t[self.ID-1]) or (WP_new != self.WP[self.ID-1]):
                print(f'Rover{self.ID} Update')
                
                self.p[self.ID-1] = p_new
                self.p_sent = self.p[self.ID-1]
                self.t[self.ID-1] = t_new
                self.WP[self.ID-1] = WP_new
                self.send_data = {}
                self.send_data ['ID'] = self.ID
                self.send_data['INo'] = self.intersection_ID
                self.send_data ['P'] = self.p[self.ID-1]
                self.send_data ['WP'] = self.WP[self.ID-1]
                self.send_data ['B'] = self.buffer[self.ID-1]
                self.send_data ['tAP'] = self.time_at_AP[self.ID-1]
                self.send_data ['tWP'] = self.time_at_WP[self.ID-1]
                for k in range(1,13):
                    self.send_data[k] = self.t[self.ID-1][k-1]

                self.collision_avoidance_all()


    def react(self,message):
        super(RoverAgent, self).react(message)
        display_message(self.aid.localname, "Rover{} erhalten: {}".format(self.ID, message.content))
        sender = message.sender

        try:
            # Save received data from other Rover
            positive_elements = []
            received_data = json.loads(message.content)
            if received_data:
                key = 'INo'
                if key in received_data:
                    if int(received_data[key]) == self.intersection_ID:     # only process data, if it affects the current intersection (if the rover has already left the intersection then, it is not his current intersection anymore)

                        key = 'ID'
                        if key in received_data:
                            Rover_ID = int(received_data[key])
                        key = 'P'
                        if key in received_data:
                            self.p[Rover_ID-1] = float(received_data[key])
                        key = 'WP'
                        if key in received_data:
                            self.WP[Rover_ID-1] = received_data[key]
                        key = 'IL'
                        if key in received_data:
                            if received_data[key] == 'True':        #Rover has left the intersection --> all data will be reset
                                print(f'Reset Rover{Rover_ID}')
                                self.p[Rover_ID-1] = 0
                                self.WP[Rover_ID-1] = None
                                self.buffer[Rover_ID-1] = None
                                self.sd[Rover_ID-1] = None
                                self.position[Rover_ID-1] = None
                                self.time_at_AP[Rover_ID-1] = None
                                for k in range(12):
                                    self.t[Rover_ID-1][k] = None
                                self.excluded_agents.append(message.sender)     # rover does not need to be informed any more 
                                return
                        key = 'B'
                        if key in received_data:
                            self.buffer[Rover_ID-1] = float(received_data[key])
                        key = 'tAP'
                        if key in received_data:
                            self.time_at_AP[Rover_ID-1] = float(received_data[key])
                        
                        key = 'tWP'
                        if key in received_data:
                            self.time_at_WP[Rover_ID-1] = float(received_data[key])

                        for k in range(12):
                            key = str(k+1)
                            if key in received_data:
                                if received_data[key] is not None:
                                    self.t[Rover_ID-1][k] = float(received_data[key])
                                    positive_elements.append(self.t[Rover_ID-1][k])      # all KPs that are used, are stored in positive_elements to calculate the starting dircetion of the rover
                                else:
                                    self.t[Rover_ID-1][k] = None    # If no time for KP k is sent, then the Rover will not pass KP k
                        
                        if self.sd[Rover_ID-1] is None:       # before the position of a new rover is calculated, all other positions are recalculated to rule out errors due to communication errors
                            for i in range(number_of_rover):      
                                number = 1
                                if self.sd[i] is not None:    # Calculate Position only for Rovers whose Starting Direction is already known
                                    for j in range(number_of_rover):                
                                        if (i != j) and (self.sd[i] == self.sd[j]):
                                            if self.time_at_AP[i] > self.time_at_AP[j]:
                                                number = max(number, self.position[j]+1)
                                    self.position[i] = number
                                    print(f'Rover {i+1} Position: {self.position[i]}')
                                
                        if positive_elements:
                            first_kp = (self.t[Rover_ID-1].index(min(positive_elements)))+1   # The start direction is determined based on the first KP used
                            #if self.sd[Rover_ID-1] is None:
                            if first_kp == 4 or first_kp == 5:
                                self.sd[Rover_ID-1] = 'N'
                            elif first_kp == 1 or first_kp == 6:
                                self.sd[Rover_ID-1] = 'O'
                            elif first_kp == 2 or first_kp == 7:
                                self.sd[Rover_ID-1] = 'S'
                            elif first_kp == 3 or first_kp == 8:
                                self.sd[Rover_ID-1] = 'W'
                                # based on the starting direction and the time at AP the position of the Rover in the line can be calculated
                            number = 1
                            for i in range(number_of_rover):
                                if (i != (Rover_ID-1)) and (self.sd[i] == self.sd[Rover_ID-1]):
                                    if self.time_at_AP[i] < self.time_at_AP[Rover_ID-1]:
                                        number = max(number, self.position[i]+1)
                            self.position[Rover_ID-1] = number

                        print(f'Rover{Rover_ID} SD: {self.sd[Rover_ID-1]} Position: {self.position[Rover_ID-1]} Priority: {self.p[Rover_ID-1]}')
                        print(f'Rover{self.ID} SD: {self.sd[self.ID-1]} Position: {self.position[self.ID-1]} Priority: {self.p[self.ID-1]}')
                        
                        # adjust own priority to avoid deadlock or congestion:
                        i = self.ID -1
                        for j in range(number_of_rover):
                            if i != j:
                                if self.sd[i] is not None and self.sd[j] is not None:                 
                                    if (self.sd[i] == self.sd[j]):
                                        if self.position[i] < self.position[j]:         # if there is an other rover from the same direction with a higher position number, the own priority has to be higher than the priority of the rover behind
                                            p_new = max(self.p[i],self.p[j])
                                            if p_new != self.p[i]:
                                                self.p[i] = p_new
                                                self.send_data['P'] = self.p[i]
                                            print(f'ROVER{i+1}: new priority:{self.p[i]}')

                    self.collision_avoidance_react(Rover_ID,sender)

        except:
            return


    def collision_avoidance_react(self,Rover_ID,sender):
        F_new = 0  # 1 if rover 1 got the approval

        # time at WP is cannot be smaller than time at WP from rover in front (no collision avoidance is needed if that is the case)
        if self.ID != (Rover_ID):
            if (self.sd[self.ID-1] == self.sd[Rover_ID-1]):
                if self.position[self.ID-1] > self.position[Rover_ID-1] and self.time_at_WP[self.ID-1] < self.time_at_WP[Rover_ID-1]:
                    self.F = 0
                    print(f'Rover{self.ID} F = {self.F}')
                    filename = os.path.join(script_directory,'CSV',f'APPROVAL{self.ID}.csv')    
                    with open(filename, mode='w', newline='', encoding='utf-8') as file:
                        writer = csv.writer(file)
                        writer.writerow([self.F])
                    print(f'Rover{self.ID} F = {self.F}')
                    return

        # collision avoidance algorithm as reaction to new times of other rover
        for k in range(12):
            #for i in range(1, self.number_of_rover +1):
            i = (self.ID-1)
            j = (Rover_ID-1)
            if i != j:
                if self.t[i][k] is not None  and self.t[j][k] is not None:
                    if (self.t[i][k] - self.buffer[i] <= self.t[j][k] + self.buffer[j]) and (self.t[i][k] + self.buffer[i] >= self.t[j][k] - self.buffer[j]):
                        if self.WP[i] == 'True': #if a rover has already driven across the WP, then the cannot be withdrawn
                                self.G[i,k] = 1
                                self.G[j,k] = 0

                        elif self.WP[j] == 'True':
                                self.G[i,k] = 0
                                self.G[j,k] = 1
                        else:
                            if self.p[i] > self.p[j]:   # rover with higher priority has right of way
                                self.G[i,k] = 1
                                self.G[j,k] = 0
                            elif self.p[i] < self.p[j]:
                                self.G[i,k] = 0
                                self.G[j,k] = 1
                            else:
                                if self.time_at_AP[i] > self.time_at_AP[j]: # if Priorities are equal -> FCFS (use the time at the AP, as the time at the AP remains unchanged in contrast to the time at the WP)
                                    self.G[i,k] = 0
                                    self.G[j,k] = 1
                                elif self.time_at_AP[i] < self.time_at_AP[j]:
                                    self.G[i,k] = 1
                                    self.G[j,k] = 0
                                else:                       # if times are equal -> the rover with the lower ID has right of way (extremly unlikely but theoretical possible)
                                    if i > j: 
                                        self.G[i,k] = 0
                                        self.G[j,k] = 1
                                    elif i < j:
                                        self.G[i,k] = 1
                                        self.G[j,k] = 0
                    else:
                            self.G[i,k] = 1
                            self.G[j,k] = 1
                else:
                    self.G[i,k] = 1
                    self.G[j,k] = 1

        self.F_i[j] = int(np.all(self.G[i]))    # approval in relation to rover j
        self.F = int(np.all(self.F_i))          # calculate the total approval taking into account all other rover
        self.F_other[j] = int(np.all(self.G[j]))
        print(f'Rover{self.ID} F = {self.F_i}')
        print(f'Rover {j+1} F = {self.F_other[j]}')
        if self.F == 1 and self.F_other[Rover_ID-1] == 0:   # if F == 1 and F of the requesting Rover == 0, send own times to requesting rover / if F == 0 own times must be adjusted -> no answer to requesting rover
            self.send_message(sender)
        filename = os.path.join(script_directory,'CSV',f'APPROVAL{self.ID}.csv')    
        with open(filename, mode='w', newline='', encoding='utf-8') as file:
            writer = csv.writer(file)
            writer.writerow([self.F])


    def collision_avoidance_all(self):
        F_new = 0  # 1 if rover 1 got the approval

        # time at WP is cannot be smaller than time at WP from rover in front (no collision avoidance is needed if that is the case)
        for i in range(number_of_rover):
            if (self.ID-1) != i and self.sd[i] is not None:
                if (self.sd[self.ID-1] == self.sd[i]):
                    if self.position[self.ID-1] > self.position[i] and self.time_at_WP[self.ID-1] < self.time_at_WP[i]:
                        self.F = 0
                        print(f'Rover{self.ID} F = {self.F}')
                        filename = os.path.join(script_directory,'CSV',f'APPROVAL{self.ID}.csv')    
                        with open(filename, mode='w', newline='', encoding='utf-8') as file:
                            writer = csv.writer(file)
                            writer.writerow([self.F])
                        return

        # collision avoidance algorithm
        for k in range(12):
            i = (self.ID-1)
            self.G[i,k] = 1
            for j in range(number_of_rover):
                if i != j:
                    if self.t[i][k] is not None  and self.t[j][k] is not None:
                        if (self.t[i][k] - self.buffer[i] <= self.t[j][k] + self.buffer[j]) and (self.t[i][k] + self.buffer[i] >= self.t[j][k] - self.buffer[j]):
                            if self.WP[i] == 'True': #if a rover has already driven across the WP, then the approval cannot be withdrawn
                                    self.G[i,k] = min(1, self.G[i,k])   
                            elif self.WP[j] == 'True':
                                    self.G[i,k] = 0
                            else:
                                if self.p[i] > self.p[j]:   # rover with higher priority has right of way
                                    self.G[i,k] = min(1, self.G[i,k])
                                elif self.p[i] < self.p[j]:
                                    self.G[i,k] = 0
                                else:
                                    if self.time_at_AP[i] > self.time_at_AP[j]: # if Priorities are equal -> FCFS (use the time at the AP, as the time at the AP remains unchanged in contrast to the time at the WP)
                                        self.G[i,k] = 0
                                    elif self.time_at_AP[i] < self.time_at_AP[j]:
                                        self.G[i,k] = min(1, self.G[i,k])
                                    else:                       # if times are equal -> the rover with the lower ID has right of way (extremly unlikely but theoretical possible)
                                        if i > j: 
                                            self.G[i,k] = 0
                                        elif i < j:
                                            self.G[i,k] = 1
                        else:
                            self.G[i,k] = min(1, self.G[i,k])
                    else:
                        self.G[i,k] = min(1, self.G[i,k])

        self.F = int(np.all(self.G[i]))
        print(f'Rover{self.ID} F = {self.F}')
        if self.F == 1:          
            self.send_broadcast_message(self.excluded_agents)
            self.F_i =  np.ones(number_of_rover)
                   
        filename = os.path.join(script_directory,'CSV',f'APPROVAL{self.ID}.csv')    
        with open(filename, mode='w', newline='', encoding='utf-8') as file:
            writer = csv.writer(file)
            writer.writerow([self.F])
    

    def send_broadcast_message(self, excluded_agents=None):
        if excluded_agents is None:
            excluded_agents = []
        data_string = json.dumps(self.send_data)
        message = ACLMessage(ACLMessage.INFORM)
        message.set_protocol(ACLMessage.FIPA_REQUEST_PROTOCOL)

        for aid in self.all_agents_aid:
            if aid != self.aid and aid not in excluded_agents:
                message.add_receiver(aid)
                self.number_of_sent_messages += 1
        message.set_content(data_string)
        self.send(message)

    def send_message(self,sender):
        data_string = json.dumps(self.send_data)
        message = ACLMessage(ACLMessage.INFORM)
        message.set_protocol(ACLMessage.FIPA_REQUEST_PROTOCOL)
        message.add_receiver(sender)
        message.set_content(data_string)
        self.send(message)
        self.number_of_sent_messages += 1




if __name__ == '__main__':
    agents = list()
    port = int(20000)
    Sim_Agent_name = 'SimAgent@localhost:{}'.format(port)
    Sim_Agent = SimAgent(AID(name=Sim_Agent_name))
    agents.append(Sim_Agent)

    #Creaion of Rover Agents
    all_agents_aid = [AID(name='Rover_{}@localhost:{}'.format(port+(1000*(i+1)),port+(1000*(i+1))))for i in range(number_of_rover)]
    print(all_agents_aid)

    c = 1000
    for i in range(number_of_rover):
        id = i+1
        port = int(20000) + c
        #port = int(argv[1]) + c
        Rover_Agent_name = 'Rover_{}@localhost:{}'.format(port,port)
        Rover_Agent = RoverAgent(AID(name=Rover_Agent_name),id, all_agents_aid)
        agents.append(Rover_Agent)
        c += 1000

    start_loop(agents)