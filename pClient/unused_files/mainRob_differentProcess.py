import sys
from croblink import *
import math
import xml.etree.ElementTree as ET
from  math import cos,sin
import numpy as np

CELLROWS=7
CELLCOLS=14
global  moved,logs

moved=False
logs={}

class MyRob(CRobLinkAngs):
    def __init__(self, rob_name, rob_id, angles, host):
        CRobLinkAngs.__init__(self, rob_name, rob_id, angles, host)
    # In this map the center of cell (i,j), (i in 0..6, j in 0..13) is mapped to labMap[i*2][j*2].
    # to know if there is a wall on top of cell(i,j) (i in 0..5), check if the value of labMap[i*2+1][j*2] is space or not

        #estimated x,y,theta caltulated by odom
        self.estx=0.0
        self.esty=0.0
        self.estdir=0.0

        self.x=0.0
        self.y=0.0
        self.theta=0.0
        
        
        self.initx=0
        self.inity=0
        self.linesensors=[]
        #1=left,2=right,3=back,4=front
        
            
    def setMap(self, labMap):
        self.labMap = labMap

    def printMap(self):
        for l in reversed(self.labMap):
            print(''.join([str(l) for l in l]))

    def run(self):
        global linesensors
        self.movement=list()
        if self.status != 0:
            print("Connection refused or error")
            quit()

        state = 'stop'
        stopped_state = 'run'
        print("connected and running")
        

        while True:
            
            self.readSensors()
            line_measure=remove_noise(self, self.measures.lineSensor)
            line_measure = [int(i) for i in line_measure]
            self.linesensors.append(line_measure)
            
            if self.measures.endLed:
                print(self.robName + " exiting")
                quit()

            if state == 'stop' and self.measures.start:
                state = stopped_state

            if state != 'stop' and self.measures.stop:
                stopped_state = state
                state = 'stop'

            if state == 'run':
                if self.measures.visitingLed==True:
                    state='wait'
                if self.measures.ground==0:
                    self.setVisitingLed(True);
                self.main(line_measure)
            elif state=='wait':
                self.setReturningLed(True)
                if self.measures.visitingLed==True:
                    self.setVisitingLed(False)
                if self.measures.returningLed==True:
                    state='return'
                self.driveMotors(0.0,0.0)
            elif state=='return':
                if self.measures.visitingLed==True:
                    self.setVisitingLed(False)
                if self.measures.returningLed==True:
                    self.setReturningLed(False)
                self.main(line_measure)
   
   
    def main(self,line_measure):
        global moved
        # DETERMINE BEST ACTION 
        action=self.determine_action()
        
        # MOVEMENT
        
                      
        
        self.move(action,line_measure)
                      
        
        moved=False
        
        #UPDATE MAP
        
         
        #TODO: make map function
        
        # self.readSensors()
        # line_measure=remove_noise(self, self.measures.lineSensor)
        # line_measure = [int(i) for i in line_measure]
        # self.linesensors.append(line_measure)
        

    def determine_action(self):
        #from linesensor readings determine action
        global moved,logs
        compass=self.measures.compass
        if compass < 0:
            compass=360 + compass
    
        #logs have the following format {action:action position:[x,y,th],line_measure:line_measure, compass:compass} 
        #Given the line sensor readings and a dict with the logs, return 5 possible actions
        # 1, for rotating +90
        # 2, for rotating -90
        # 3, for rotating +180
        # 4, for moving forward 2 units
        
        #linesensors may not have 10 values, so we check
        if len(self.linesensors) <= 10:
            action= 4
            return action
        elif all(self.linesensors[-10:][3:6]):
            #means that the robot still can go forward even more. 
            action=4
            # print("moving forward")
            return action
        elif not any(self.linesensors):
        #means that there isnt any line in front of the robot, so it has to check the 10 previous values of linesenors readings to see if theres any line 
        # to the left 
            if any(self.linesensors[-10:][0][0:3]):
                #to the left -90
                action=1
                print("turning left")
                return action
            elif any(self.linesensors[-10:][0][4:7]):
                # to the right(-90)
                print("turning right")
                action=2
                return action
            
            else:
                #means nothing was seen, rotating 180
                action=3
                print("turning 180")
                return action 



    def move(self,action,line_measure): 
        global moved,logs
        compass=self.measures.compass
        if compass < 0:
            compass=360 + compass
        self.linesensors=[]
        #initial positions on the step
        x,y,init_th=self.movement_model(0,0)
        self.initx=x
        self.inity=y
        
        if action==4:
            print("moving forward")
        elif action==1:
            print("turning left")
        elif action==2:
            print("turning right")
        elif action==3:
            print("turning 180")

                           
        logs=logging(self,logs,x,y,line_measure,compass,action)
        
        
        while not moved:
                
            
            # 1, for rotating +90
            if action==1:
                if not moved:
                    moved,x,y=self.rotate_P(90,compass,moved)
                if moved: 
                    print("in position,stopped")
                    return x,y
                
            # 2, for rotating -90
            elif action==2:
                if not moved:
                    moved,x,y=self.rotate_N(-90,compass,moved)
                if moved:
                    print("in position,stopped")
                    return x,y
                
            # 3, for rotating +180
            elif action==3:
                if not moved:
                    moved,x,y=self.rotate_P(180,compass,moved)
                if moved:
                    print("in position,stopped")
                    return x,y
                
            # 4, for moving forward 2 units
            elif action==4:
                
                if not moved :
                    moved,x,y=self.move_forward(2,x,y,compass,line_measure)
                if moved :
                    print("in position,stopped")
                    return x,y
                
        
            
            self.readSensors()
            line_measure=remove_noise(self, self.measures.lineSensor)
            line_measure = [int(i) for i in line_measure]
            self.linesensors.append(line_measure)
            
        
         
    
  
    def move_forward(self,units,x,y,compass,line_measure):
        #moving forward units units
        global moved,linesensors    
        
        # moveTo is the x,y coordinates i want to move to based on thye compass angle
        
        self.estx=x
        self.esty=y

        # x,y,th=forwardConstraints(self,line_measure)
        ## MOVEMENT FROM PERCEPTION AND CONTROL(analitical movement) ## 
        # forward movements
        if all(line_measure):
            self.driveMotors(0.04,0.04)
            x,y,th=self.movement_model(0.05,0.05)
            
            
        if all(line_measure[2:5]):
            # print("moving forward ")
            self.driveMotors(0.04,0.04)
            x,y,th=self.movement_model(0.04,0.04)
            
            #left movements
        elif all(line_measure[0:3]):
            self.movement.append("left")
            # print("turning left ")
            self.driveMotors(-0.01,0.01)
            x,y,th=self.movement_model(-0.01,0.01)
            
        # right movements
        elif all(line_measure[4:7]):
            # print("turning right ")
            self.driveMotors(0.01,-0.01)
            x,y,th=self.movement_model(0.01,-0.01)
            


        print(abs(x-self.initx),abs(y-self.inity))
        #check final positions to see if in finalPos 
        if abs(x-self.initx)>=2 or abs(y-self.inity)>=2:
            moved=True
            #updating xy coordinates so that when it reaches 2, it removes noise 
            x,y,th=self.movement_model(0,0)
            self.driveMotors(0,0)
            
        return moved,x,y
        
        
        
        
        
    def rotate_P(self,target_angle,compass,inPosition):
        #rotating positively(right to left )
        angle_threshold=4
        
        if abs(compass - target_angle)<= angle_threshold:
            self.driveMotors(0,0)
            inPosition=True
            x,y,th=self.movement_model(0,0)
        else: 
            
            self.driveMotors(-0.01,0.01)
            x,y,th=self.movement_model(-0.01,0.01)

        return inPosition,x,y
        
        
    def rotate_N(self,target_angle,compass,inPosition):
        #rotating negatively(left to right )
        angle_threshold=4

            
        if abs(compass - target_angle)<= angle_threshold:
            self.driveMotors(0,0)
            inPosition=True
            x,y,th=self.movement_model(0,0)
        else: 
            
            self.driveMotors(0.01,-0.01)
            x,y,th=self.movement_model(0.01,-0.01)
            
        return inPosition,x,y
        
        

   

    def movement_model(self,motorL,motorR):
        # to know how much the robot moved and where it is and its orientation.
        lin= (motorL+motorR)/2
        rot=(motorR-motorL)/2
        
        #new position values
        x=self.estx+lin*cos(self.estdir)
        y=self.esty+lin*sin(self.estdir)
        theta=self.estdir+rot
        #updating values
        self.x=x
        self.y=y
        self.estx=x
        self.esty=y
        self.estdir = theta
        
        if self.estdir < 0:
            self.estdir += 2 * math.pi
        elif self.estdir > 2 * math.pi:
            self.estdir -= 2 * math.pi  
        # return the orientation in degrees
        theta_deg = theta * 180 / math.pi

        return x,y,theta_deg
        
        
        
    

def logging(self,logs,x,y,line_measure,compass,action):
    global moved
    #adds dict values to the log dict
    #values of all crossroads
    #logs["position"]= x,y of the robot
    #logs["action"]=action taken
    #logs["TDactio"]= all actions not taken,TODOactions.
    #TODOactions are the actions that were not taken but were possible to take
    #Possible actions must be all calculated IF the robot has compass=0.
    
    if moved:
        logs["position"]=[x,y]
    
    logs["position"] = [x, y]

    # Calculate and update TODOactions if necessary
    if compass == 0:
        possible_actions = calculate_possible_actions(self,self.linesensors,line_measure,compass)  # Example function to calculate possible actions
        todo_actions = [act for act in possible_actions if act != action]
        logs["TDactio"] = todo_actions

    # Additional logging logic if needed

    # Return the updated logs
    return logs

def calculate_possible_actions(self,linesensor,line_measure,compass):
    global moved
    possible_actions=[]
    # Calculate the possible actions based on the linesensor and the crossroads it is on. 
    
    first_array = linesensor[0]  # Get the first array
    # Calculate the difference between the first array and all the other arrays
    diff = linesensor[1:] - first_array

    # checking all values that arent 0
    differences = []
    for i, row in enumerate(diff):
        indices = [j for j, val in enumerate(row) if val != 0]
        differences.append(indices)
    #remove from nonzero_indices all the empty lists

    # print(differences)

    #remove from differences all the empty lists
    nemptydiff = [x for x in differences if x != []]

    #if nemptydiff is none, or lines forward, turn back
    if nemptydiff == []:
        for i, row in enumerate(differences):
            if 0 in row:
                possible_actions.append(2)
            if 6 in row:
                possible_actions.append(1)

    elif all(line_measure[0:3]):
        possible_actions.append(4)
    else:
        possible_actions.append(3)  
        #nothing at the end, turn back
    

    
    return possible_actions

def convert_action_to_0degree_action(self,action,compass):
    #converts the action to a 0 degree action
    #4=forwards,1=left,2=right,3=turn around
    orientations={
        0:{4:4,1:1,2:2,3:3},
        90:{4:1,1:3,2:4,3:2},
        180:{4:3,1:2,2:1,3:4},
        270:{4:2,1:4,2:3,3:1}}

    #checking which orientation is closest now
    possible_ori=[abs(compass-0),abs(compass-90),abs(compass-180),abs(compass-270)]
    ori_now=possible_ori.index(min(possible_ori))
    
    #returning the action that corresponds to the 0 degree action
    return orientations[ori_now][action]
    
    
    
    
    
    
    
    
    
    
    

def remove_noise(self,line_measure):
    #de perceção e controlo
    window_size = 10
    line_measure = [int(i) for i in line_measure]
    read_lines.append(line_measure)

    if len(read_lines) ==window_size:
        result = [sum(i)/window_size for i in zip(*read_lines[-window_size:])]
        result = [1 if i > 0.5 else 0 for i in result]
        return result
        
    else :
        return line_measure

class Map():
    def __init__(self, filename):
        tree = ET.parse(filename)
        root = tree.getroot()
        
        self.labMap = [[' '] * (CELLCOLS*2-1) for i in range(CELLROWS*2-1) ]
        i=1
        for child in root.iter('Row'):
           line=child.attrib['Pattern']
           row =int(child.attrib['Pos'])
           if row % 2 == 0:  # this line defines vertical lines
               for c in range(len(line)):
                   if (c+1) % 3 == 0:
                       if line[c] == '|':
                           self.labMap[row][(c+1)//3*2-1]='|'
                       else:
                           None
           else:  # this line defines horizontal lines
               for c in range(len(line)):
                   if c % 3 == 0:
                       if line[c] == '-':
                           self.labMap[row][c//3*2]='-'
                       else:
                           None
               
           i=i+1


rob_name = "pClient1"
host = "localhost"
pos = 1
mapc = None
read_lines=[]
for i in range(1, len(sys.argv),2):
    if (sys.argv[i] == "--host" or sys.argv[i] == "-h") and i != len(sys.argv) - 1:
        host = sys.argv[i + 1]
    elif (sys.argv[i] == "--pos" or sys.argv[i] == "-p") and i != len(sys.argv) - 1:
        pos = int(sys.argv[i + 1])
    elif (sys.argv[i] == "--robname" or sys.argv[i] == "-r") and i != len(sys.argv) - 1:
        rob_name = sys.argv[i + 1]
    elif (sys.argv[i] == "--map" or sys.argv[i] == "-m") and i != len(sys.argv) - 1:
        mapc = Map(sys.argv[i + 1])
    else:
        print("Unkown argument", sys.argv[i])
        quit()

if __name__ == '__main__':
    rob=MyRob(rob_name,pos,[0.0,60.0,-60.0,180.0],host)
    if mapc != None:
        rob.setMap(mapc.labMap)
        rob.printMap()
    
    rob.run()
