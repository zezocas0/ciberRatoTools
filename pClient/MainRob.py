import sys
from croblink import *
import math
import xml.etree.ElementTree as ET
from  math import cos,sin,radians, pi,degrees
import numpy as np
import random







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
        self.linedistance=0.438
        
        self.initx=0
        self.inity=0
        self.linesensors=[]
        self.previousaction=[]
        #1=left,2=right,3=back,4=front
        
            
    def setMap(self, labMap):
        self.labMap = labMap

    def printMap(self):
        for l in reversed(self.labMap):
            print(''.join([str(l) for l in l]))

    def run(self):
        global moved,logs
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
            compass=self.measures.compass
            if compass<0:
                compass=360+compass
            
            #Initial log that is needed
            logs = logging(self, logs, self.x, self.y, line_measure, compass, action=None)
            # print("logs",logs)
    
            
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
                self.main(line_measure,compass)
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
                self.main(line_measure,compass)
   
   
    def main(self,line_measure,compass):
        global moved,logs
       
       


       
        # DETERMINE BEST ACTION 
        action=self.determine_action(line_measure,self.x,self.y)
        print("New Action! ",action)
        
        # MOVEMENT 
        
        self.x,self.y,line_measure,compass=self.move(action,line_measure)

        logs = logging(self, logs, self.x, self.y, line_measure, compass, action)
        print("last 2 logs",  list(logs.keys())[-1:])
        moved=False
        
        #UPDATE MAP
        
         
        #TODO: make map function
        
        self.readSensors()
        line_measure=remove_noise(self, self.measures.lineSensor)
        line_measure = [int(i) for i in line_measure]
        

            

    def determine_action(self, line_measure, x, y):
        # From linesensor readings determine action
        global moved, logs
        compass = self.measures.compass
        if compass < 0:
            compass = 360 + compass

        
        #from last log, determine action from "todo_actions"
        keys=list(logs.keys())
        (x,y)=keys[-1]
        possible_movements=logs[(x,y)]['todo_actions']
        
        if possible_movements!=[]:
            #prioritize going forward
            if 4 in possible_movements:
                possible_action=4
            else:
                possible_action=random.choice(possible_movements)
        else:
            all_movements=logs[(x,y)]['done_actions']
            possible_action=random.choice(all_movements)
            
        #need to see the action_to_0degree_action 

        action=convert_action_to_0degree_action(self,possible_action,compass)
        
        return action







    '''
        if len(logs) == 0:  # If no logs, see if it can go forward
            if any(line_measure[3:6]):
                # Means that the robot can go forward
                action = 4
                return action
            else:
                # If the robot can't go forward, go left until we can
                action = 1
                return action
        else:
            dega0 = convert_action_to_0degree_action(self, 4, compass)

            if (dega0, x, y) in logs:
                if dega0 in logs[(dega0, x, y)]['done_actions']:
                    # The robot has already moved forward in this position
                    # Check if it can move based on todo_actions of logs in this position
                    if not len(logs[(dega0, x, y)]['todo_actions']) == 0:
                        # There are todo_actions, choose a random action from todo_actions
                        action = random.choice(logs[(dega0, x, y)]['todo_actions'])
                    else:
                        # If it can't move forward or rotate, rotate 180
                        action = 3
                        return action
                else:
                    # The robot hasn't moved forward in this position, so we move forward
                    action = 4

                    if not len(logs[(dega0, x, y)]['todo_actions']) == 0:
                        # There are todo_actions, choose a random action from todo_actions
                        action = random.choice(logs[(dega0, x, y)]['todo_actions'])
                    else:
                        # If it can't move forward or rotate, rotate 180
                        action = 3
                        return action
            else:
                # The robot hasn't moved forward in this position, so we move forward
                action = 4

        return action

      '''              


    def move(self,action,line_measure): 
        global moved,logs
        th=self.measures.compass
        if th < 0:
            th=360 + th
               
        #initial positions on the step
         # coordinates based on the logs, to kindof "remove" the noise from motion.
        if len(logs)==0:    
            x,y,init_th=self.motion_model(0,0)
            self.initx=x
            self.inity=y
        else:
            #checking last position in the logs
            # if -1 in logs and 'position' in logs[-1]:
            #     self.estx = logs[-1]['position'][0]
            #     self.esty= logs[-1]['position'][1]
            keys = list(logs.keys())
            (x,y)= keys[-1]
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

           
        while not moved:
                
            
            # 1, for rotating +90
            if action==1:
                if not moved:
                    moved,x,y,th=self.rotate_P(90,th,moved,line_measure)
                if moved: 
                    print("in position,stopped")
                    # print(compass)
                    # return x,y,th,line_measure
                    break
            # 2, for rotating -90
            elif action==2:
                if not moved:
                    moved,x,y,th=self.rotate_N(90,th,moved,line_measure)
                if moved:
                    print("in position,stopped")
                    # print(compass)
                    # return x,y,th,line_measure
                    break
                
            # 3, for rotating +180
            elif action==3:
                if not moved:
                    moved,x,y,th=self.rotate_P(180,th,moved,line_measure)
                if moved:
                    print("in position,stopped")
                    # print(compass)
                    # return x,y,th,line_measure
                    break
            # 4, for moving forward 2 units
            elif action==4:
                
                if not moved :
                    moved,x,y=self.move_forward(x,y,th,line_measure)
                if moved :
                    print("in position,stopped")
                    # print(x,y)
                    return x,y,th,line_measure

        
        
            self.readSensors()
            line_measure=remove_noise(self, self.measures.lineSensor)
            line_measure = [int(i) for i in line_measure]
            self.linesensors.append(line_measure)
            
            th=self.measures.compass
            if th < 0:
                th=360 + th
        
        #if it moves and last action isnt 4, then it means it rotated, and needs to move forward
        if moved and action!=4:
            moved=False
            while not moved:
                if not moved:
                    moved,x,y=self.move_forward(x,y,th,line_measure)
                if moved:
                    print("in position,stopped")
                    # print(x,y)
                    return x,y,th,line_measure
        

        
                self.readSensors()
                line_measure=remove_noise(self, self.measures.lineSensor)
                line_measure = [int(i) for i in line_measure]
                self.linesensors.append(line_measure)
                
                th=self.measures.compass
                if th < 0:
                    th=360 + th
            
         
    
  
    def move_forward(self,x,y,compass,line_measure):
        #moving forward units units
        global moved,logs  
        
        self.estx=x
        self.esty=y
        

        # x,y,th=forwardConstraints(self,line_measure)
        ## MOVEMENT FROM PERCEPTION AND CONTROL(analitical movement) ## 
        # forward movements
        if all(line_measure):
            self.driveMotors(0.04,0.04)
            x,y,th=self.motion_model(0.04,0.04)  

        elif all(line_measure[2:5]):
            #forward movements
            self.driveMotors(0.04,0.04)
            x,y,th=self.motion_model(0.04,0.04)
            #left movements
        elif all(line_measure[0:3]):
            self.driveMotors(-0.01,0.01)
            x,y,th=self.motion_model(-0.01,0.01)
        # right movements
        elif all(line_measure[4:7]):
            self.driveMotors(0.01,-0.01)
            x,y,th=self.motion_model(0.01,-0.01)
        else:
            #move forward
            self.driveMotors(0.01,0.01)
            x,y,th=self.motion_model(0.01,0.01)
            
            
        # print(abs(x-self.initx),abs(y-self.inity))
        #check final positions to see if in finalPos 
        if abs(x-self.initx)>=2 or abs(y-self.inity-self.linedistance)>=2:
            #make sure that x and y are even
            x=round(x)
            y=round(y)
            moved=True
            #updating xy coordinates so that when it reaches 2, it removes noise 
            x,y,th=self.motion_model(0,0)
            self.driveMotors(0,0)
            
        return moved,x,y
        
        
        
        
        
    def rotate_P(self,target_angle,th,moved,line_measure):
        #rotating positively(right to left )
        angle_threshold=4
        # print(compass)
        
        if abs(th - target_angle)<= angle_threshold:
            moved=True
            x,y,th=self.motion_model(0,0)
            self.driveMotors(0,0)
            
            if all(line_measure[2:5]):
            #to check if it is perfectly aligned
                return moved,x,y,th
            
            
        else: 
            
            self.driveMotors(-0.01,0.01)
            x,y,th=self.motion_model(-0.01,0.01)

        return moved,x,y,th
        
        
    def rotate_N(self,target_angle,th,moved,line_measure):
        #rotating negatively(left to right )
        angle_threshold=4

            
        if abs(th - target_angle)<= angle_threshold:
            moved=True
            x,y,th=self.motion_model(0,0)
            self.driveMotors(0,0)
            if all(line_measure[2:5]):
                return moved,x,y,th
        else: 
            
            self.driveMotors(0.01,-0.01)
            x,y,th=self.motion_model(0.01,-0.01)
            
        return moved,x,y,th
        
        

   

    def motion_model(self,motorL,motorR):
        # to know how much the robot moved and where it is and its orientation.
        noise=(random.gauss(1,0.15**2),random.gauss(1,0.15**2))



        lin= (motorL+motorR)/2
        rot= (motorR-motorL)/1 
        
        #new position values
        x=self.estx+lin*cos(self.estdir)
        y=self.esty+lin*sin(self.estdir)
        
        theta=self.estdir+rot
        
        # convert theta to radians and ensure it is within the range of 0 to 2pi
        theta = theta % (2 * pi)
        theta = radians(theta)
        
        #updating values
        self.x=x
        self.y=y    
        #turn theta to degrees
        th=degrees(theta)
        
        return x,y,th
        
        
        
    
def logging(self, logs, x, y, line_measure, compass, action=None):
    # Adds dict values to the log dict
    # Values of all crossroads
    
    if action==None and x==0 and y==0:
    #at 0,0 in the beginning theres no action. its only to log the only options
        if (x, y) not in logs:
            logs[(x, y)] = {"done_actions": [], "todo_actions": []}
            # print("Adding new position to logs")
        # Calculate the possible actions based on the line sensor and compass

        possible_actions = calculate_possible_actions(self,line_measure, compass)

        todo_actions = [a for a in possible_actions if a not in logs[(x, y)]["done_actions"]]
        logs[(x, y)]["todo_actions"] = todo_actions
        # Update the todo_actions list for the current position

        #action is from pervious position, so we add it to the done_actions from -2 logs
        #keys -1 may not exist, so we need to check if it exists
        # Update the done_actions list of previous position, and remove the action from the todo_actions list

           
    
    
    if moved:
        line_measure = remove_noise(self, self.measures.lineSensor)
        line_measure = [int(i) for i in line_measure]
        
        x=round(x)
        y=round(y)
        print(x,y)
        self.previousaction=action
        # Create a new dict for the current position if it doesn't exist
        if (x, y) not in logs:
            logs[(x, y)] = {"done_actions": [], "todo_actions": []}
            # print("Adding new position to logs")
        # Calculate the possible actions based on the line sensor and compass

        possible_actions = calculate_possible_actions(self,line_measure, compass)

        todo_actions = [a for a in possible_actions if a not in logs[(x, y)]["done_actions"]]
        logs[(x, y)]["todo_actions"] = todo_actions
        # Update the todo_actions list for the current position

        #action is from pervious position, so we add it to the done_actions from -2 logs
        
        keys = list(logs.keys())
        (x_old,y_old)= keys[-2]
        #keys -1 may not exist, so we need to check if it exists

        self.previousaction=action        
        if (x_old,y_old) in logs:
            
            # Update the done_actions list of previous position, and remove the action from the todo_actions list
            
            # print("done logs",logs[(x_old,y_old)]["done_actions"])
            
            # print(logs[(x_old,y_old)]["done_actions"])
            # print(logs  [(x_old,y_old)]["todo_actions"])
            # print("previous action", self.previousaction)
            
            logs[(x_old,y_old)]["todo_actions"].remove(self.previousaction)
            logs[(x_old,y_old)]["done_actions"].append(self.previousaction)
    
        print(logs)
        
    # Return the updated logs
    return logs


def calculate_possible_actions(self,line_measure,compass):
    #from the linesensor, we calculate all possible actions when the robot is on a crossroad
    global moved
    possible_actions=[]
    
    # Calculate the difference between the first array and all the other arrays
    diff = np.diff(self.linesensors, axis=0)
    
    # checking all values that arent 0
    differences = []
    for i, row in enumerate(diff):
        if isinstance(row, np.ndarray):
            indices = [j for j, val in enumerate(row) if val != 0]
            differences.append(indices)
    nemptydiff = [x for x in differences if x != []]

    
    #checks if it can go left or right
    for row in nemptydiff:
        if 0 in row:
            possible_actions.append(1)
        if 6 in row:
            possible_actions.append(2)
    
    if  all(line_measure[2:5]):
        possible_actions.append(4)
    else:
        possible_actions.append(3)  
        #nothing at the end, turn back
    #it should always be able to turn backwards
    possible_actions.append(3)
     
    #no repeating values
    possible_actions = list(set(possible_actions))
   
    # print("possibleactions",possible_actions)
    return possible_actions

def convert_action_to_0degree_action(self,action,compass):
    #converts the action to a 0 degree action
    #4=forwards,1=left,2=right,3=turn around
    orientations={ 0:{4:4,1:1,2:2,3:3},90:{4:1,1:3,2:4,3:2},180:{4:3,1:2,2:1,3:4},270:{4:2,1:4,2:3,3:1}}

    #checking which orientation is closest now
    possible_ori=[abs(compass-0),abs(compass-90),abs(compass-180),abs(compass-270),abs(compass-360)]
    ori_now=[0,90,180,270,0][possible_ori.index(min(possible_ori))]
    
    try:
        # print("ori",orientations[ori_now][action])
        possible_actions=orientations[ori_now][action]
        return possible_actions
    
    except KeyError as e:
        #TODO: to see if this still makes error
        print(f"keyerror {e}")
        print(ori_now,action)    
        print(possible_ori)
    
    
    
    
    
    
    
    

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
