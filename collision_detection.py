import numpy as np
import csv
import os
from sys import argv

class IM:
    def __init__(self,number_of_rover):
        self.number_N = 0
        self.number_O = 0
        self.number_S = 0
        self.number_W = 0
        self.sd= [None]*number_of_rover
        self.position = [None]*number_of_rover


    def IM_Collision_Avoidance(self,number_of_rover,script_directory, sim_time):
        t = np.zeros((number_of_rover,12))   #time when rover i is at KP k
        p = np.zeros(number_of_rover)    # priority of rover i
        WP = [None]*number_of_rover #time at WP for rover i
        #A = np.zeros(number_of_rover)   --> wird gerade nicht gebraucht
        #C = np.zeros((number_of_rover,number_of_rover,12))  --> wird gerade nicht gebraucht
        G = np.ones((number_of_rover,number_of_rover,12))        # 1 if rover i is the winner over rover j regarding KP k
        F = np.zeros(number_of_rover)    # 1 if rover i has an approval
        #sd = [None]*number_of_rover #starting directions of rover


        number_of_rover = number_of_rover
        buffer = 1
        #Red Data:
        for i in range(1,number_of_rover+1):
            filename = os.path.join(script_directory,'CSV',f'KPS{i}.csv')

            read_data = []
        
            with open(filename, mode='r', newline='', encoding='utf-8') as file:
                reader = csv.DictReader(file)

                for row in reader:
                    read_data.append(row)
            if read_data:
                key = 'P'
                if key in read_data[0]:
                    p[i-1] = read_data[0][key]

                key = 'WP'
                if key in read_data[0]:
                    WP[i-1] = read_data[0][key]

                for k in range(1,13):
                    key = str(k)
                    if key in read_data[0]:

                        t[i-1,k-1] = read_data[0][key]

        #create starting directions and positions in line of rover:
        first_kp = []
        for row in t:
            positive_elements = [value for value in row if value > 0]
            if not positive_elements:
                first_kp.append(None)
                continue
            min_positive_element=min(positive_elements)
            min_index=row.tolist().index(min_positive_element)
            first_kp.append(min_index+1)
        #print(first_kp)
        
        for i in range(1, number_of_rover+1):
            if self.sd[i-1] is None:
                if first_kp[i-1] == 4 or first_kp[i-1] == 5:
                    self.sd[i-1] = 'N'
                    self.number_N += 1
                    self.position[i-1] = self.number_N
                elif first_kp[i-1] == 1 or first_kp[i-1] == 6:
                    self.sd[i-1] = 'O'
                    self.number_O += 1
                    self.position[i-1] = self.number_O
                elif first_kp[i-1] == 2 or first_kp[i-1] == 7:
                    self.sd[i-1] = 'S'
                    self.number_S += 1
                    self.position[i-1] = self.number_S
                elif first_kp[i-1] == 3 or first_kp[i-1] == 8:
                    self.sd[i-1] = 'W'
                    self.number_W += 1
                    self.position[i-1] = self.number_W

        #print(f'Fahrzeug Herkunft: {self.sd}')
        #print(f'Anzahl Fahrzeuge aus N: {self.number_N}\n Anzahl Fahrzeuge aus O: {self.number_O}\n Anzahl Fahrzeuge aus S: {self.number_S}\n Anzahl Fahrzeuge aus W: {self.number_W}\n')

        #adjust priorities of rover:
        for i in range(1, number_of_rover+1):
            for j in range(1, number_of_rover+1):
                if i != j:
                    if self.sd[i-1] is not None and self.sd[j-1] is not None:                    
                        if (self.sd[i-1] == self.sd[j-1]):
                            if self.position[i-1] < self.position[j-1]:
                                p[i-1] = max(p[i-1],p[j-1]+1)
        #print(p)
                                

        #Collision Avoidance Algorithm:
        for k in range(1,13):
            for i in range(1, number_of_rover +1):
                for j in range(1, number_of_rover +1):
                    if i != j:
                        if t[i-1,k-1] > 0 and t[j-1,k-1] > 0:
                            if (t[i-1,k-1] - buffer <= t[j-1,k-1] + buffer) and (t[i-1,k-1] + buffer >= t[j-1,k-1] - buffer):
                                if WP[i-1] == True: #if a rover has already driven across the WP, then the cannot be withdrawn
                                    G[i-1,j-1,k-1] = 1
                                    G[j-1,i-1,k-1] = 0
                                elif WP[j-1] == True:
                                    G[i-1,j-1,k-1] = 0
                                    G[j-1,i-1,k-1] = 1
                                else:                            
                                    #C[i-1,j-1,k-1] = 1
                                    if p[i-1] > p[j-1]:
                                        G[i-1,j-1,k-1] = 1
                                        G[j-1,i-1,k-1] = 0
                                    elif p[i-1] < p[j-1]:
                                        G[i-1,j-1,k-1] = 0
                                        G[j-1,i-1,k-1] = 1
                                    else:
                                        if t[i-1,k-1] > t[j-1,k-1]:
                                            G[i-1,j-1,k-1] = 0
                                            G[j-1,i-1,k-1] = 1
                                        elif t[i-1,k-1] < t[j-1,k-1]:
                                            G[i-1,j-1,k-1] = 1
                                            G[j-1,i-1,k-1] = 0
                                        else:       # Fall wenn sowohl Prio, als auch Zeitpunkt gleich ist -> muss ich noch etwas überlegen, ist aber sehr unwahrscheinlich
                                            G[i-1,j-1,k-1] = 1
                                            G[j-1,i-1,k-1] = 0
                            else:
                                #C[i-1,j-1,k-1] = 0
                                G[i-1,j-1,k-1] = 1
                                G[j-1,i-1,k-1] = 1
                        else:
                                #C[i-1,j-1,k-1] = 0
                                G[i-1,j-1,k-1] = 1
                                G[j-1,i-1,k-1] = 1
        for i in range(1, number_of_rover +1):
            subset=G[i-1]
            F[i-1] = int(np.all(subset))
    
        #print(f'Gewinner:\n {G}')
        #print(f'Freigabe: \n {F}')
        #print(f'Zeiten:\n {t}')
        #print(f'Konflikte: \n {C}') 
        #Send Approvals:
        for i in range(1,number_of_rover+1):
            filename = os.path.join(script_directory,'CSV',f'APPROVAL{i}.csv')
            with open(filename, mode='w', newline='', encoding='utf-8') as file:
                writer = csv.writer(file)
                writer.writerow([F[i-1]])

