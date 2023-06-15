import sys
from croblink import *
from math import *
import xml.etree.ElementTree as ET
import math
import numpy as np

CELLROWS=7
CELLCOLS=14
global movements ,rotated,moved,step,moveTo
movements=[]
rotated=False
moved=False
step=0
moveTo=[0,0]
class MyRob(CRobLinkAngs):
    def __init__(self, rob_name, rob_id, angles, host):
        CRobLinkAngs.__init__(self, rob_name, rob_id, angles, host)
    # In this map the center of cell (i,j), (i in 0..6, j in 0..13) is mapped to labMap[i*2][j*2].
    # to know if there is a wall on top of cell(i,j) (i in 0..5), check if the value of labMap[i*2+1][j*2] is space or not

        #estimated x,y,theta caltulated by odom
        self.estx=0.0
        self.esty=0.0
        self.estdir=0.0

        self.dx=0
        self.dy=0
        self.dtheta=0

        self.x=0.0
        self.y=0.0
        self.theta=0.0
        
        
        self.initx=0
        self.inity=0
        
        
            
    def setMap(self, labMap):
        self.labMap = labMap

    def printMap(self):
        for l in reversed(self.labMap):
            print(''.join([str(l) for l in l]))

    def run(self):
        self.movement=list()
        if self.status != 0:
            print("Connection refused or error")
            quit()

        state = 'stop'
        stopped_state = 'run'
        print("connected and running")
        while True:
            self.readSensors()


            # print("line", self.measures.lineSensor)
            # print("ground", self.measures.ground)

       
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
                self.move()
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
                self.move()


    def move(self): 
        global movements,rotated,moved,step
        compass=self.measures.compass
        if compass < 0:
            compass=360 + compass


        
        line_measure=remove_noise(self, self.measures.lineSensor)
        line_measure = [int(i) for i in line_measure]
        
        #initial positions on the step
        x,y,init_th=self.movement_model(0,0)
        self.initx=x
        self.inity=y
        
        # #rotating positively 
        '''
        if not inPosition:
            inPosition=self.rotate_P(180,compass,inPosition)
        if inPosition:
            print("in position,stopped")
        print("compass angle", compass)
        # print("estimated angle", th)
        '''        
       
       
        # #rotating negatively  
        # if not rotated and step==0:
        #     rotated=self.rotate_N(270,compass,rotated)
        # if rotated and step==0:
        #     step+=1
        #     print("in position,stopped")
        # # print("compass angle", compass)
        # # print("estimated angle", th)
        
        
        
        #moving forward 2 units if not movc
        if not moved and step==0:
     
            moved,x,y=self.move_forward(2,x,y,compass,line_measure)
        
        if moved and step==0:
            step+=1
            print("in position,stopped")
        # print(f"movement in x,y : {x},{y}")
        
        
        
        return 
    
        
    def move_forward(self,units,x,y,compass,line_measure):
        #moving forward units units
        global moved,moveTo    

        
        # moveTo is the x,y coordinates i want to move to based on thye compass angle

            

        # x,y,th=forwardConstraints(self,line_measure)
        ## MOVEMENT FROM PERCEPTION AND CONTROL(analitical movement) ## 
        #left movements
        if all(line_measure[0:3]):
            self.movement.append("left")
            print("turning left ")
            self.driveMotors(-0.05,0.05)
            x,y,th=self.movement_model(-0.05,0.05)
            movements.append("left")
        # forward movements
        elif all(line_measure[3:6]) or all(line_measure[2:5]) or all (line_measure[1:4]) or all(line_measure[1:6]):
            print("moving forward ")
            self.driveMotors(0.15,0.15)
            x,y,th=self.movement_model(0.15,0.15)
            movements.append("forward")
        # right movements
        elif all(line_measure[4:7]):
            print("turning right ")
            self.driveMotors(0.05,-0.05)
            x,y,th=self.movement_model(0.05,-0.05)
            movements.append("right")

        # backward movements
        elif not any(line_measure):
            print("moving backward ")
            self.driveMotors(-0.13,-0.13)
            x,y,th=self.movement_model(-0.13,-0.13)
            movements.append("backward")
        else:
            # if any other case, moves forward slowly
            print("moving forward - slow ")
            self.driveMotors(-0.15,0.15)
            x,y,th=self.movement_model(-0.15,0.15)
            movements.append("forward")


        #check final positions to see if in finalPos 
        if abs(x)>=abs(self.initx+2) or abs(y)>=abs(self.inity+2):
            moved=True
            

        return moved,x,y
        
    def rotate_P(self,target_angle,compass,inPosition):
        #rotating positively(right to left )
        angle_threshold=4
        
        if abs(compass - target_angle)<= angle_threshold:
            self.driveMotors(0,0)
            inPosition=True
        else: 
            
            self.driveMotors(-0.01,0.01)
            x,y,th=self.movement_model(-0.01,0.01)
            print(th)
        return inPosition
        
        
    def rotate_N(self,target_angle,compass,inPosition):
        #rotating negatively(left to right )
        angle_threshold=4

            
        if abs(compass - target_angle)<= angle_threshold:
            self.driveMotors(0,0)
            inPosition=True
        else: 
            
            self.driveMotors(0.01,-0.01)
            x,y,th=self.movement_model(0.01,-0.01)
            print(th)
        return inPosition
        
        

   

    def movement_model(self,motorL,motorR):
        # to know how much the robot moved and where it is and its orientation.
        lin= (motorL+motorR)/2
        rot=(motorR-motorL)/2
        
        #new position values
        x=self.estx+lin*cos(self.estdir)
        y=self.esty+lin*sin(self.estdir)
        theta=self.estdir+rot


        self.estdir = theta
        
        if self.estdir < 0:
            self.estdir += 2 * math.pi
        elif self.estdir > 2 * math.pi:
            self.estdir -= 2 * math.pi  
            
        # return the orientation in degrees
        theta_deg = theta * 180 / math.pi
        self.estdir_deg = self.estdir * 180 / math.pi

        return x,y,theta_deg
        
        

        
        
        
        
        
    def map_memory(self):
        
        #to remember the map and where the robot has been as to not repeat the path if theres new ones unexplored
        #to know where the robot is in the map
        #starts in positon 0,0. matrix with values. every odd number is a number from self.measures.ground_sensor. either 0,-1,1,2,.. every even number is a line. | if the line is vertical and -if its horizontal. 
        
        x=int(self.estx)
        y=int(self.esty)
        #to know where the robot is facing
        theta=self.estdir
        #to know if the robot is facing up or down
                
        if 100>theta>80 or -80>theta>-100:
                    #to know if the robot is facing up
                    if theta>0:
                        #to know if the robot is facing up and if its in a new cell or just in the middle of the cell
                        if self.dx>=3:
                            #to know if the robot is in a new cell and if the cell is empty
                            if self.labMap[y-1][x]==' ':
                                #to know if the robot is in a new cell and if the cell is empty and if the robot is facing up. if so, add a | to the map
                                self.labMap[y-1][x]='|'
                            #to know if the robot is in a new cell and if the cell is not empty and if the robot is facing up. if so, add a X to the map
                            else:
                                self.labMap[y-1][x]='X'
                        #to know if the robot is in the middle of the cell and if the cell is empty
                        elif self.labMap[y][x]==' ':
                            #to know if the robot is in the middle of the cell and if the cell is empty and if the robot is facing up. if so, add a | to the map
                            self.labMap[y][x]='|'
                        #to know if the robot is in the middle of the cell and if the cell is not empty and if the robot is facing up. if so, add a X to the map
                        else:
                            self.labMap[y][x]='X'
                    #to know if the robot is facing down
                    else:
                        #to know if the robot is facing down and if its in a new cell or just in the middle of the cell
                        if self.dx>=3:
                            #to know if the robot is in a new cell and if the cell is empty
                            if self.labMap[y+1][x]==' ':
                                #to know if the robot is in a new cell and if the cell is empty and if the robot is facing down. if so, add a | to the map
                                self.labMap[y+1][x]='|'
                            #to know if the robot is in a new cell and if the cell is not empty and if the robot is facing down. if so, add a X to the map
                            else:
                                self.labMap[y+1][x]='X'
                        #to know if the robot is in the middle of the cell and if the cell is empty
                        elif self.labMap[y][x]==' ':
                            #to know if the robot is in the middle of the cell and if the cell is empty and if the robot is facing down. if so, add a | to the map
                            self.labMap[y][x]='|'
                        #to know if the robot is in the middle of the cell and if the cell is not empty and if the robot is facing down. if so, add a X to the map
                        else:
                            self.labMap[y][x]='X'



    
    


def forwardConstraints(self, line_measure):
    global movements
    ## MOVEMENT FROM PERCEPTION AND CONTROL(analitical movement) ## 
    #left movements
    if all(line_measure[0:3]):
        self.movement.append("left")
        print("turning left ")
        self.driveMotors(-0.05,0.05)
        x,y,th=self.movement_model(-0.05,0.05)
        movements.append("left")
    # forward movements
    elif all(line_measure[3:6]) or all(line_measure[2:5]) or all (line_measure[1:4]) or all(line_measure[1:6]):
        print("moving forward ")
        self.driveMotors(0.15,0.15)
        x,y,th=self.movement_model(0.15,0.15)
        movements.append("forward")
    # right movements
    elif all(line_measure[4:7]):
        print("turning right ")
        self.driveMotors(0.05,-0.05)
        x,y,th=self.movement_model(0.05,-0.05)
        movements.append("right")

    # backward movements
    elif not any(line_measure):
        print("moving backward ")
        self.driveMotors(-0.13,-0.13)
        x,y,th=self.movement_model(-0.13,-0.13)
        movements.append("backward")
    else:
        # if any other case, moves forward slowly
        print("moving forward - slow ")
        self.driveMotors(-0.15,0.15)
        x,y,th=self.movement_model(-0.15,0.15)
        movements.append("forward")

    return x,y,th


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
