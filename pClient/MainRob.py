import sys
from croblink import *
import math
import xml.etree.ElementTree as ET
from  math import cos,sin,radians, pi,degrees
import numpy as np
import random

from lib.pid import PIDController






CELLROWS=7
CELLCOLS=14
global  moved,logs

moved=False
logs={}
map=[]

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
        
        kp = 0.05138938976206109
        ki = 0.0022398605506287416
        kd = 0.0008512325205835791
        self.pid_controller=PIDController(kp,ki,kd)

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
            logs = logging(self, logs, self.x, self.y,self.theta, line_measure, action=None)
            # print("logs",logs)

            map=self.update_map(self.x,self.y,self.theta,logs,line_measure)
            
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
        
        self.x,self.y,self.theta,line_measure=self.move(action,line_measure)

        logs = logging(self, logs, self.x, self.y,self.theta, line_measure, action)
        print("last 2 logs",  list(logs.keys())[-1:])
        moved=False
        
        #UPDATE MAP
        
         
        #TODO: make map function
        map=self.update_map(self.x,self.y,self.theta,logs,line_measure)
        
        self.readSensors()
        line_measure=remove_noise(self, self.measures.lineSensor)
        line_measure = [int(i) for i in line_measure]
        

            

    def determine_action(self, line_measure, x, y):
        # From linesensor readings determine action
        global moved, logs,map
        backwards=False
        compass = self.measures.compass
        if compass < 0:
            compass = 360 + compass

        
        #from last log, determine action from "todo_actions"
        
        keys=list(logs.keys())
        (x,y)=keys[-1]
        possible_movements=logs[(x,y)]['todo_actions']


        if possible_movements != []:
            # prioritize going forward
            if 4 in possible_movements:
                possible_action = 4
                possible_movements.remove(4)
            # then left or right
            elif 1 in possible_movements or 2 in possible_movements:
                possible_actions = [1, 2]
                possible_actions = [action for action in possible_actions if action in possible_movements]
                if possible_actions:
                    possible_action = random.choice(possible_actions)
                    possible_movements.remove(possible_action)
            # if there's no 1, 2, or 4, now I can go backwards
            elif 3 in possible_movements:
                possible_action = 3
                possible_movements.remove(3)

        else:
            all_movements=logs[(x,y)]['done_actions']
            possible_action=random.choice(all_movements)
            
        #need to see the action_to_0degree_action 
        #TODO: change this to logging to save the actions into the 0 degree action. 
  
        
        action=convert_0degree_action_to_action(self,possible_action,compass)
        
        return action



         


    def move(self,action,line_measure): 
        global moved,logs
               
        #initial positions on the step
         # coordinates based on the logs, to kindof "remove" the noise from motion.
        if len(logs)==0:    
            x,y,init_th=self.movement_model(0,0)
            self.initx=x
            self.inity=y
            self.initth=init_th
            th=self.initth
        else:
            #checking last position in the logs
            # if -1 in logs and 'position' in logs[-1]:
            #     self.estx = logs[-1]['position'][0]
            #     self.esty= logs[-1]['position'][1]
            keys = list(logs.keys())
            (x,y)= keys[-1]
            self.initx=x
            self.inity=y
            self.initth=logs[(x,y)]['th']
            th=self.initth
        
                    
        
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
                target_angle=90+self.initth
                if target_angle>=360:
                    target_angle=target_angle-360
                
                if not moved:
                    moved,x,y,th=self.rotate_P(target_angle,th,moved,line_measure)
                if moved: 
                    print("in position,stopped")
                    # print(compass)
                    # return x,y,th,line_measure
                    break
            # 2, for rotating -90
            elif action==2:
                target_angle=self.initth-90
                if target_angle<0:
                    target_angle=target_angle+360
                    
                if not moved:
                    moved,x,y,th=self.rotate_N(target_angle,th,moved,line_measure)
                    
                if moved:
                    print("in position,stopped")
                    # print(compass)
                    # return x,y,th,line_measure
                    break
                
            # 3, for rotating +180
            elif action==3:
                target_angle=180+self.initth
                if target_angle>=360:
                    target_angle=target_angle-360
                
                if not moved:
                    moved,x,y,th=self.rotate_P(target_angle,th,moved,line_measure)
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
                
            
        return x,y,th,line_measure
    
  
    def move_forward(self,x,y,th,line_measure):
        #moving forward units units
        global moved,logs  
        
        self.estx=x
        self.esty=y
        est_th=th
        pid_controller=self.pid_controller
        
        self.estx=x
        self.esty=y
        est_th=th        
        #calling the class PIDController
        if abs(x-self.initx)<=1 and abs(y-self.inity)<=1:
            error = 0.0
            for i in range(len(line_measure)):
                error += (i - 3) * line_measure[i]
            #try catch cause sum(line_measure) can be 0
            try:
                error /= sum(line_measure)
            except:
                error=0.0
            # Update the PID controller with the error, time step calculated inside the pid_controller.update 
            control_output = pid_controller.update(error)
            # drive motors using control_output

            self.driveMotors(0.04+control_output,0.04-control_output)
            x,y,th=self.movement_model(0.04+control_output,0.04-control_output)
            
        else :
            self.driveMotors(0.04,0.04) 
            x,y,th=self.movement_model(0.04,0.04)
            
         
     
            
        # print(abs(x-self.initx),abs(y-self.inity))
        # print(x,y,th)
        #check final positions to see if in finalPos 
        if abs(x-self.initx)>=2 or abs(y-self.inity)>=2:
            #make sure that x and y are even
            x=round(x)
            y=round(y)
            moved=True
            #updating xy coordinates so that when it reaches 2, it removes noise 
            x,y,th=self.movement_model(0,0)
            self.driveMotors(0,0)
            
        return moved,x,y
        
        
        
        
        
    def rotate_P(self,target_angle,th,moved,line_measure):
        #rotating positively(right to left )
        angle_threshold=0.8

        # print(compass)
        
        if abs(th - target_angle)<= angle_threshold:
            
            moved=True
            x,y,th=self.movement_model(0,0)
            self.driveMotors(0,0)
            
            if all(line_measure[2:5]):
            #to check if it is perfectly aligned
                return moved,x,y,th
            
            
        else: 
            
            self.driveMotors(-0.01,0.01)
            x,y,th=self.movement_model(-0.01,0.01)

        return moved,x,y,th
        
        
    def rotate_N(self,target_angle,th,moved,line_measure):
          #rotating positively(right to left )
        angle_threshold=1
        # print(compass)
        if abs(th - target_angle)<= angle_threshold:
            moved=True
            x,y,th=self.movement_model(0,0)
            self.driveMotors(0,0)
            
            if all(line_measure[2:5]):
            #to check if it is perfectly aligned
                return moved,x,y,th
            
            
        else: 
            
            self.driveMotors(0.01,-0.01)
            x,y,th=self.movement_model(0.01,-0.01)

        return moved,x,y,th
   

    def movement_model(self,motorL,motorR):
        # to know how much the robot moved and where it is and its orientation.
        noise=(random.gauss(1,0.15**2),random.gauss(1,0.15**2))



        lin= (motorL+motorR)/2
        rot= (motorR-motorL)/1 
        t_last=self.estdir
        #new position values
        
        x=self.estx+lin*cos(t_last)
        y=self.esty+lin*sin(t_last)
        
        theta=t_last+rot
        # convert theta to radians and ensure it is within the range of 0 to 2pi
        if theta > 2*pi:
            theta = theta - 2*pi
            
        
        #updating values
        self.x=x
        self.y=y    
        self.estdir=theta
        #turn theta to degrees
        th=degrees(theta)
        
        
        return x,y,th
            
    def update_map(self, x, y, theta, logs, line_measure):
        global map
        all_actions = []

    
        # obtain last logs
        keys = list(logs.keys())
        last_log = keys[-1]
        # obtain the tdodo and done actions of the last log
        all_actions = logs[last_log]["todo_actions"] + logs[last_log]["done_actions"]


        if(map == []) or  (last_log[0]==0 and last_log[1]==0) :
            # first map update
            # create a matrix of all ' ' value cells with a size of 21*
            map = [[' ' for _ in range(49)] for _ in range(21)]
            # put 0 in the middle of the map, coordinates 11, 25
            point0 = 11 * 49 + 25
            zeroy = point0 % 49
            zerox = point0 // 49


            # update map with the first position of the robot
            map[zerox][zeroy] = '0'


            # define the directions
            for i in all_actions:
                if i == 1:
                #if 1(left), add | above the zerox, zeroy
                    map[zerox-1][zeroy] = '|'
                
                if i==2:
                #if 2(right),add | below the zerox, zeroy
                    map[zerox+1][zeroy] = '|'
                if i==3:
                #if 3(behind), add - to the left of zerox, zeroy
                    map[zerox][zeroy-1] = '-'
                if i==4:
                #if 4(forward), add - to the right of zerox, zeroy
                    map[zerox][zeroy+1] = '-'
                    
                
                
                
        else:
            
            point0 = 11 * 49 + 25
            zeroy = point0 % 49
            zerox = point0 // 49
            
            # obtain last logs
            keys = list(logs.keys())
            last_log = keys[-1]
            # obtain the tdodo and done actions of the last log
            all_actions = logs[last_log]["todo_actions"] + logs[last_log]["done_actions"]
            log_ground_sensor=logs[last_log]["beacons"]
                
            
            logx=last_log[1]
            logy=last_log[0]
            beaconx=zerox+logx
            beacony=zeroy+logy
            
            #to see if its in a beacon
            if log_ground_sensor!= [-1]:
                map[beaconx][beacony]=log_ground_sensor
            
            
            # define the directions
            for i in all_actions:
                if i == 1:
                #if 1(left), add | above the zerox, zeroy
                    map[beaconx-1][beacony] = '|'
                
                if i==2:
                #if 2(right),add | below the zerox, zeroy
                    map[zerox+1][beacony] = '|'
                if i==3:
                #if 3(behind), add - to the left of zerox, zeroy
                    map[beaconx][beacony-1] = '-'
                if i==4:
                #if 4(forward), add - to the right of zerox, zeroy
                    map[beaconx][beacony+1] = '-'
                
             
            # print(logx,logy)
            # print(zerox,zeroy)
            
            # print("updating map")   
            # for i in range(21):
            #     print(map[i])    
                
        # Save the map to a file
        
        with open("mapping.out", "w") as file:
            for row in map:
                file.write(' '.join(str(cell) for cell in row) + '\n')

        
        

        return

    
      
    
def logging(self, logs, x, y,th, line_measure, action=None):
    # Adds dict values to the log dict
    # Values of all crossroads
    
    if action==None and x==0 and y==0:
    #at 0,0 in the beginning theres no action. its only to log the only options
        if (x, y) not in logs:
            logs[(x, y)] = {"done_actions": [], "todo_actions": [],"beacons":[],"th":th}
            # print("Adding new position to logs")
        # Calculate the possible actions based on the line sensor and compass

        possible_actions = calculate_possible_actions(self,line_measure, th)

        todo_actions = [a for a in possible_actions if a not in logs[(x, y)]["done_actions"]]
        logs[(x, y)]["todo_actions"] = todo_actions
        # Update the todo_actions list for the current position

        #action is from pervious position, so we add it to the done_actions from -2 logs
        #keys -1 may not exist, so we need to check if it exists
        # Update the done_actions list of previous position, and remove the action from the todo_actions list
        ground_sensor=self.measures.ground
        logs[(x,y)]["beacons"].append(ground_sensor)
            
 
    
    
    if moved:
        line_measure = remove_noise(self, self.measures.lineSensor)
        line_measure = [int(i) for i in line_measure]
        
        x=round(x)
        y=round(y)
        print(x,y)
        self.previousaction=action
        # Create a new dict for the current position if it doesn't exist
        if (x, y) not in logs:
            logs[(x, y)] = {"done_actions": [], "todo_actions": [],"beacons":[],"th":th}
            # print("Adding new position to logs")
        # Calculate the possible actions based on the line sensor and compass

        possible_actions = calculate_possible_actions(self,line_measure, th)

        todo_actions = [a for a in possible_actions if a not in logs[(x, y)]["done_actions"]]
        logs[(x, y)]["todo_actions"] = todo_actions
        # Update the todo_actions list for the current position
        ground_sensor=self.measures.ground
        logs[(x,y)]["beacons"].append(ground_sensor)
            
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
            try:
                logs[(x_old,y_old)]["todo_actions"].remove(self.previousaction)
                
            except:
                print("----error----")
                print("error trying to remove previous action(624)")    
                print(x_old,y_old)
                print(logs[(x_old,y_old)]["todo_actions"])
                print(self.previousaction)
            
                print("----error----") 
            
            
            
            logs[(x_old,y_old)]["done_actions"].append(self.previousaction)
    
        print(logs)
        
    # Return the updated logs
    return logs


def calculate_possible_actions(self,line_measure,th):
    #from the linesensor, we calculate all possible actions when the robot is on a crossroad
    global moved
    possible_actions=[]; possible_actions_in0=[]
    
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
    
    #no repeating values
    possible_actions = list(set(possible_actions))
    
    #for all possible_actions, convert them to when the action would be at 0 degrees
    for i in range(len(possible_actions)):
 
        # do something with possible_actions[i]
        action=convert_action_to_0degree_action(self,possible_actions[i],th)
        possible_actions_in0.append(action)
 
    # print("possibleactions",possible_actions)
    return possible_actions_in0

def convert_action_to_0degree_action(self,action,th):
    #converts the action to a 0 degree action
    #4=forwards,1=left,2=right,3=turn around
    # orientations={ 
    #               0:{4:4,1:1,2:2,3:3},
    #               90:{4:1,1:3,2:4,3:2},
    #               180:{4:3,1:2,2:1,3:4},
    #               270:{4:2,1:4,2:3,3:1}
    #               }

    orientations={ 
                  0:{4:4,1:1,2:2,3:3},
                  90:{4:2, 1:4 , 2:3 , 3:1},
                  180:{4:3,1:2,2:1,3:4},
                  270:{4:1,1:3,2:4,3:2}
                  }

    #checking which orientation is closest now
    possible_ori=[abs(th-0),abs(th-90),abs(th-180),abs(th-270),abs(th-360)]
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
        
def convert_0degree_action_to_action(self, action, th):
    # Converts the 0 degree action to the original action
    # 4=forwards, 1=left, 2=right, 3=turn around
    # orientations = {
    #     0: {4: 4, 1: 1, 2: 2, 3: 3},
    #     90: {4: 2, 1: 4, 2: 2, 3: 1},
    #     180: {4: 3, 1: 2, 2: 1, 3: 4},
    #     270: {4: 1, 1: 3, 2: 4, 3: 2}
    # }
    orientations = {
        0:   {4:4 , 1:1 , 2: 2 , 3:3 },
        90:  {4:1 , 1:4 , 2:3 , 3:2  },
        180: {4:3 , 1:2 , 2:1 , 3:4  },
        270: {4:2 , 4:1 , 3:2 , 1:3  }
    }

    # Checking which orientation is closest now
    possible_ori = [abs(th - 0), abs(th - 90), abs(th - 180), abs(th - 270), abs(th - 360)]
    ori_now = [0, 90, 180, 270, 0][possible_ori.index(min(possible_ori))]

    try:
        # print("orientations",orientations)
        # print("ori now",ori_now)
        # print("action",action)
        possible_actions=orientations[ori_now][action]
        return possible_actions
    

    except KeyError as e:
        print(f"KeyError {e}")
        print(ori_now, action)
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
