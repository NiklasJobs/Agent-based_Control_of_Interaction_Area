from pade.misc.utility import display_message, start_loop, call_later
from pade.acl.messages import ACLMessage
from pade.core.agent import Agent
from pade.acl.aid import AID
from pade.behaviours.protocols import FipaRequestProtocol
from pade.behaviours.protocols import TimedBehaviour
import json
from constants import *

from twisted.internet import reactor
import csv
import numpy as np
import os
import subprocess
from sys import argv

script_directory = os.path.dirname(os.path.abspath(__file__))
number_of_rover = NUMBER_OF_ROVERS
number_of_messages = 0 # variable to count the number of sent messages

for i in range(1, number_of_rover+1):  
    file = os.path.join(script_directory,'CSV',f'KPS{i}.csv')
    with open(file, mode='w', newline='', encoding='utf-8') as file:
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
            writer.writerow(['hierarchical'])
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

#############################################################################################################################################
# Timed Behavior from Rover Agent
class CheckKPSBehavior(TimedBehaviour):
    def __init__(self,agent,time):
        super(CheckKPSBehavior,self).__init__(agent,time)
        
    def on_time(self):
        super(CheckKPSBehavior,self).on_time()
        self.agent.check_KPS()
#################################################################
# Request Behavior from Rover Agent
class RequestRover(FipaRequestProtocol):
    def __init__(self,agent,message):
        super(RequestRover,self).__init__(agent=agent,message=message,is_initiator=True)


##################################################################
class RoverAgent(Agent):
    def __init__(self, aid, ID):
        super(RoverAgent, self).__init__(aid=aid, debug=False)
        self.ID = ID
        self.p = 0
        self.t =[None]*12
        self.WP = False
        self.il = False
        self.buffer = None
        self.approval = None
        self.send_data = {}
        self.time_at_AP = None
        self.time_at_WP = None
        self.number_of_messages = 0


    def on_start(self):
        super(RoverAgent, self).on_start()
        display_message(self.aid.localname, "Rover Agent gestartet!")
        display_message(self.aid.localname,"{}".format(AID))
        # self.check_KPS()
        check_kps_behavior = CheckKPSBehavior(self,1.0)
        self.behaviours.append(check_kps_behavior)

        message = None
        self.request_behavior = RequestRover(self, message)
        self.behaviours.append(self.request_behavior)


    def react(self, message):
        super(RoverAgent,self).react(message)
        display_message(self.aid.localname, "Rover{} erhalten: {}".format(self.ID,message.content))
        if message.sender.name == IM_Agent_name:
            c = message.content
            filename = os.path.join(script_directory,'CSV',f'APPROVAL{self.ID}.csv')
            with open(filename, mode='w', newline='', encoding='utf-8') as file:
                writer = csv.writer(file)
                writer.writerow([c])


    def check_KPS(self):
        p_new = 0
        t_new = [None]*12
        WP_new = False
        IL_new = False
        filename = os.path.join(script_directory,'CSV',f'KPS{self.ID}.csv')
        read_data = []
    
        with open(filename, mode='r', newline='', encoding='utf-8') as file:
            reader = csv.DictReader(file)

            for row in reader:
                read_data.append(row)
        if read_data:
            key = 'P'
            if key in read_data[0]:
                p_new = read_data[0][key]
            key = 'WP'
            if key in read_data[0]:
                WP_new = read_data[0][key]
            key = 'IL'
            if key in read_data[0]:
                IL_new = read_data[0][key]  
            key = 'B'
            if key in read_data[0]:
                self.buffer = read_data[0][key]
            key = 'tAP'
            if key in read_data[0]:
                self.time_at_AP = read_data[0][key]
            key = 'tWP'
            if key in read_data[0]:
                self.time_at_WP = read_data[0][key]  
            for k in range(1,13):
                key = str(k)
                if key in read_data[0]:
                    t_new[k-1] = read_data[0][key]

        if (IL_new == 'True'):
            self.send_data = {}
            self.send_data ['ID'] = self.ID
            self.send_data['IL'] = IL_new
            self.send_request()
            # reset own data
            self.p = 0
            self.t = [None]*12
            self.WP = False
            global number_of_messages
            number_of_messages += self.number_of_messages
            self.number_of_messages = 0


            with open(filename, mode='w', newline='', encoding='utf-8') as file:
                pass
        else:
            if (p_new != self.p) or (t_new != self.t) or (WP_new != self.WP):
                print(f'Rover{self.ID} Update')
                self.send_data = {}
                self.p = p_new
                self.t = t_new
                self.WP = WP_new
                self.send_data ['ID'] = self.ID
                self.send_data ['P'] = self.p
                self.send_data ['WP'] = self.WP
                self.send_data ['B'] = self.buffer
                self.send_data ['tAP'] = self.time_at_AP
                self.send_data ['tWP'] = self.time_at_WP
                for k in range(1,13):
                    self.send_data[k] = self.t[k-1]

                self.send_request()
        
    def send_request(self):
        #print(f'Rover{self.ID}: Sende Request')
        data_string = json.dumps(self.send_data)
        message = ACLMessage(ACLMessage.REQUEST)
        message.set_protocol(ACLMessage.FIPA_REQUEST_PROTOCOL)
        message.add_receiver(AID(name=IM_Agent_name))
        message.set_content(data_string)
        filename = os.path.join(script_directory,'CSV',f'APPROVAL{self.ID}.csv')
        with open(filename, mode='w', newline='', encoding='utf-8') as file:
            pass
        self.number_of_messages +=1
         
        self.request_behavior.message = message
        self.request_behavior.on_start()
    
        
#############################################################################################################################################
# Request / Inform Behavior of IM Agent
class ReceiveRequestIM(FipaRequestProtocol):
    def __init__(self,agent):
        super(ReceiveRequestIM,self).__init__(agent=agent,message=None,is_initiator=False)

    def handle_request(self, message):
        super(ReceiveRequestIM,self).handle_request(message)
        #print(f"IM hat eine Nachricht von {message.sender} bekommen mit Inhalt: {message.content}")
        display_message(self.agent.aid.localname, message.content)
        positive_elements = []
        received_data = json.loads(message.content)
        if received_data:
            key = 'ID'
            if key in received_data:
                Rover_ID = int(received_data[key])
            key = 'P'
            if key in received_data:
                p_new = float(received_data[key])
                self.agent.p[Rover_ID-1] =max(p_new, self.agent.p[Rover_ID-1]) #only set priority if no higher priority is set for Rover
            key = 'WP'
            if key in received_data:
                self.agent.WP[Rover_ID-1] = received_data[key]
            key = 'IL'
            if key in received_data:
                if received_data[key] == 'True':        #Rover has left the intersection --> all data will be reset
                    print(f'Reset Rover{Rover_ID}')
                    self.agent.p[Rover_ID-1] = 0
                    self.agent.WP[Rover_ID-1] = None
                    self.agent.buffer[Rover_ID-1] = None
                    self.agent.sd[Rover_ID-1] = None
                    self.agent.position[Rover_ID-1] = None
                    self.agent.time_at_AP[Rover_ID-1] = None
                    for k in range(12):
                        self.agent.t[Rover_ID-1][k] = None
                    # update number of sent messages
                    global number_of_messages
                    number_of_messages += (self.agent.number_of_messages-self.agent.old_number_of_messages)
                    self.agent.old_number_of_messages = self.agent.number_of_messages
                    return
            key = 'B'
            if key in received_data:
                self.agent.buffer[Rover_ID-1] = float(received_data[key])
            key = 'tAP'
            if key in received_data:
                self.agent.time_at_AP[Rover_ID-1] = float(received_data[key])
            
            key = 'tWP'
            if key in received_data:
                self.agent.time_at_WP[Rover_ID-1] = float(received_data[key])

            for k in range(12):
                key = str(k+1)
                if key in received_data:
                    if received_data[key] is not None:
                        self.agent.t[Rover_ID-1][k] = float(received_data[key])
                        positive_elements.append(self.agent.t[Rover_ID-1][k])       # all KPs that are used, are stored
            
            if self.agent.sd[Rover_ID-1] is None:       # before the position of a new rover is calculated, all other positions are recalculated to rule out errors due to communication errors
                for i in range(number_of_rover):      
                    number = 1
                    if self.agent.sd[i] is not None:    # Calculate Position only for Rovers whose Starting Direction is already known
                        for j in range(number_of_rover):                
                            if (i != j) and (self.agent.sd[i] == self.agent.sd[j]):
                                if self.agent.time_at_AP[i] > self.agent.time_at_AP[j]:
                                    number = max(number, self.agent.position[j]+1)
                        self.agent.position[i] = number
                        print(f'Rover {i+1} Position: {self.agent.position[i]}')
                    
            if positive_elements:
                first_kp = (self.agent.t[Rover_ID-1].index(min(positive_elements)))+1   # The start direction is determined based on the first KP used
                print(f'Rover{Rover_ID}: First KP = {first_kp}')
                if self.agent.sd[Rover_ID-1] is None:
                    if first_kp == 4 or first_kp == 5:
                        self.agent.sd[Rover_ID-1] = 'N'

                    elif first_kp == 1 or first_kp == 6:
                        self.agent.sd[Rover_ID-1] = 'O'

                    elif first_kp == 2 or first_kp == 7:
                        self.agent.sd[Rover_ID-1] = 'S'

                    elif first_kp == 3 or first_kp == 8:
                        self.agent.sd[Rover_ID-1] = 'W'

                    number = 1
                    for i in range(number_of_rover):
                        if (i != (Rover_ID-1)) and (self.agent.sd[i] == self.agent.sd[Rover_ID-1]):
                            if self.agent.time_at_AP[i] < self.agent.time_at_AP[Rover_ID-1]:
                                number = max(number, self.agent.position[i]+1)
                    self.agent.position[Rover_ID-1] = number

            self.agent.contact_information[Rover_ID-1] = message.sender.name
            print(f'Rover{Rover_ID} SD: {self.agent.sd[Rover_ID-1]} Position: {self.agent.position[Rover_ID-1]}')
            
            # adjust priorities of rover to avoid deadlock or congestion:
            for i in range(number_of_rover):
                for j in range(number_of_rover):
                    if i != j:
                        if self.agent.sd[i] is not None and self.agent.sd[j] is not None:                    
                            if (self.agent.sd[i] == self.agent.sd[j]):
                                if self.agent.position[i] < self.agent.position[j]:
                                    p_new = max(self.agent.p[i],self.agent.p[j])
                                    if p_new != self.agent.p[i]:
                                        self.agent.p[i] = p_new
                                        print(f'ROVER{i+1}: new priority:{self.agent.p[i]}')
                                       # message = ACLMessage(ACLMessage.INFORM)
                                        #message.set_protocol(ACLMessage.FIPA_REQUEST_PROTOCOL)
                                        #message.add_receiver(AID(self.agent.contact_information[i]))
                                        #message.set_content(2.0)
                                        #self.agent.send(message)
                                        #self.agent.number_of_messages +=1

        self.agent.collision_avoidance(Rover_ID)    
        reply = message.create_reply()
        reply.set_performative(ACLMessage.INFORM)
        reply.set_content(self.agent.F[Rover_ID-1])
        self.agent.send(reply)
        self.agent.number_of_messages +=1
#############################################################################################################################################

class IMAgent(Agent):
    def __init__(self, aid):
        super(IMAgent, self).__init__(aid=aid, debug=False)
        self.t = [[None for _ in range(12)] for _ in range(number_of_rover)]
        self.p = [0]*(number_of_rover)
        self.WP = [None]*number_of_rover #1 if rover i has passed the WP
        #self.t_WP = [None]*number_of_rover # time when rover i plans to pass the WP
        self.buffer = [None]*(number_of_rover) #saftey buffer for rover i
        self.G = np.ones((number_of_rover,number_of_rover,12))
        self.F = np.zeros(number_of_rover)  # 1 if rover 1 got the approval
        self.number_of_rover = number_of_rover
        self.sd = [None]*number_of_rover #start direction of Rover i
        self.time_at_AP = [None]*number_of_rover
        self.time_at_WP = [None]*number_of_rover
        self.contact_information ={}
        self.position = [None]*number_of_rover
        self.number_of_messages = 0     # current number of sent messages
        self.old_number_of_messages = 0 # number of messages at the last update

        self.receive_request = ReceiveRequestIM(self)
        self.behaviours.append(self.receive_request)




    def on_start(self):
        super(IMAgent, self).on_start()
        display_message(self.aid.localname, "Intersection Manager Agent gestartet!")
        display_message(self.aid.localname,"{}".format(AID)) 

    def collision_avoidance(self,Rover_ID):
        F_new = np.zeros(number_of_rover)  # 1 if rover 1 got the approval

        # time at WP is cannot be smaller than time at WP from rover in front (no collision avoidance is needed if that is the case)
        for i in range(number_of_rover):
            if i != (Rover_ID-1):
                if (self.sd[i] == self.sd[Rover_ID-1]):
                    if self.position[i] < self.position[Rover_ID-1] and self.time_at_WP[i]>self.time_at_WP[Rover_ID-1]:
                        self.F[Rover_ID-1] = 0
                        return

        # collision avoidance algorithm
        for k in range(12):
            #for i in range(1, self.number_of_rover +1):
            i = (Rover_ID-1)
            for j in range(self.number_of_rover):
                if i != j:
                    if self.t[i][k] is not None  and self.t[j][k] is not None:
                        if (self.t[i][k] - self.buffer[i] <= self.t[j][k] + self.buffer[j]) and (self.t[i][k] + self.buffer[i] >= self.t[j][k] - self.buffer[j]):
                            if self.WP[i] == 'True': #if a rover has already driven across the WP, then the cannot be withdrawn
                                    self.G[i,j,k] = 1
                                    self.G[j,i,k] = 0
                            elif self.WP[j] == 'True':
                                    self.G[i,j,k] = 0
                                    self.G[j,i,k] = 1
                            else:
                                if self.p[i] > self.p[j]:   # rover with higher priority has right of way
                                    self.G[i,j,k] = 1
                                    self.G[j,i,k] = 0
                                elif self.p[i] < self.p[j]:
                                    self.G[i,j,k] = 0
                                    self.G[j,i,k] = 1
                                else:
                                    if self.time_at_AP[i] > self.time_at_AP[j]: # if Priorities are equal -> FCFS (use the time at the AP, as the time at the AP remains unchanged in contrast to the time at the WP)
                                        self.G[i,j,k] = 0
                                        self.G[j,i,k] = 1
                                    elif self.time_at_AP[i] < self.time_at_AP[j]:
                                        self.G[i,j,k] = 1
                                        self.G[j,i,k] = 0
                                    else:                       # if times are equal -> the rover with the lower ID has right of way (extremly unlikely but theoretical possible)
                                        if i > j:
                                            self.G[i,j,k] = 0
                                            self.G[j,i,k] = 1
                                        elif i < j:
                                            self.G[i,j,k] = 0
                                            self.G[j,i,k] = 1

                        else:
                                self.G[i,j,k] = 1
                                self.G[j,i,k] = 1
                    else:
                        self.G[i,j,k] = 1
                        self.G[j,i,k] = 1

        for i in range(self.number_of_rover):
            subset=self.G[i]
            F_new[i] = int(np.all(subset))
            self.F[i] = F_new[i]
            if (i != Rover_ID-1) and (F_new[i] == 0):# Agent Rover_ID is informed separately in response to his request
                print(f"[DEBUG] Sending approval update to Rover{i+1} with new F: {F_new[i]}")
                message = ACLMessage(ACLMessage.INFORM)
                message.set_protocol(ACLMessage.FIPA_REQUEST_PROTOCOL)
                message.add_receiver(AID(self.contact_information[i]))
                message.set_content(self.F[i])
                self.send(message)
                self.number_of_messages +=1



if __name__ == '__main__':
    agents = list()
    port = int(20000)
    #Creation of Simulation Agent
    Sim_Agent_name = 'SimAgent@localhost:{}'.format(port)
    Sim_Agent = SimAgent(AID(name=Sim_Agent_name))
    agents.append(Sim_Agent)
    
    #Creation of Intersection Manager Agent
    c=1000
    port += c
    IM_Agent_name = 'IMAgent@localhost:{}'.format(port)
    IM_Agent = IMAgent(AID(name=IM_Agent_name))
    agents.append(IM_Agent)
    #Creaion of Rover Agents
    for i in range(number_of_rover):
        id = i+1
        port += c
        Rover_Agent_name = 'Rover{}@localhost:{}'.format(port,port)
        Rover_Agent= RoverAgent(AID(name=(Rover_Agent_name)),id)
        agents.append(Rover_Agent)

    start_loop(agents)