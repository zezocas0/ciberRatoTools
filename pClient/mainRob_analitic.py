import sys
from croblink import *
from math import *
import xml.etree.ElementTree as ET


CELLROWS=7
CELLCOLS=14
global movements 
movements=[]

class MyRob(CRobLinkAngs):
    def __init__(self, rob_name, rob_id, angles, host):
        CRobLinkAngs.__init__(self, rob_name, rob_id, angles, host)
    # In this map the center of cell (i,j), (i in 0..6, j in 0..13) is mapped to labMap[i*2][j*2].
    # to know if there is a wall on top of cell(i,j) (i in 0..5), check if the value of labMap[i*2+1][j*2] is space or not

        #estimated x,y,theta caltulated by odom
        self.estx=0.0
        self.esty=0.0
        self.estdir=0.0
        self.dx=0.0
        self.dy=0.0
        self.dtheta=0.0
            
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
                self.wander()
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
                self.wander()


    def wander(self): 
        global movements
        
        line_measure=remove_noise(self, self.measures.lineSensor)
        line_measure = [int(i) for i in line_measure]

        

        ## general motion to turn left or right
        # move robot
        self.move_robot(0.15,0.15)
        # see xy theta position after movement
        x,y,theta=self.movement_model(0.15,0.15)
        
        #check theta to see if its vertical or horizontal(with a range of 10 degrees(90and -90))
        if 100>theta>80 or -80>theta>-100:
            #see the diference between the last position and the new one
            self.dx=y-self.esty
            #if dx>=2 it means the robot is in a new cell
            if self.dx >=2:
                #check if the robot is in a new cell or just in the middle of the cell
                if self.dx >=3:
                    #if its in a new cell it means it has to turn left or right
                    if theta>0:
                        #turn left
                        self.move_robot(-0.15,0.15)
                        #update position
                        x,y,theta=self.movement_model(-0.15,0.15)
                        #update position
                        self.estx=x
                        self.esty=y
                        self.estdir=theta
                        #update map
                        self.map_memory()
                        #append movement to list
                        movements.append('l')
                    else:
                        #turn right
                        self.move_robot(0.15,-0.15)
                        #update position
                        x,y,theta=self.movement_model(0.15,-0.15)
                        #update position
                        self.estx=x
                        self.esty=y
                        self.estdir=theta
                        #update map
                        self.map_memory()
                        #append movement to list
                        movements.append('r')
                else:
                    #if its in the middle of the cell it means it has to go forward
                    self.move_robot(0.15,0.15)
                    #update position
                    x,y,theta=self.movement_model(0.15,0.15)
                    #update position
                    self.estx=x
                    self.esty=y
                    self.estdir=theta
                    #update map
                    self.map_memory()
                    #append movement to list
                    movements.append('f')
            
            
            
        pass
            



    def movement_model(self,motorL,motorR):
        # to know how much the robot moved and where it is and its orientation.
        lin= (motorL+motorR)/2
        rot=(motorR-motorL)/2
        
        #new position values
        x=self.estx+lin*cos(self.estdir)
        y=self.esty+lin*sin(self.estdir)
        theta=self.estddir+rot
        #turning theta into degrees
        theta=theta*180/pi
        
        return x,y,theta
        
        
    def move_robot(self,motorL,motorR):
        #to move the robot
        self.driveMotors(motorL,motorR)
        
        
        
        
        
        
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



    
    



def remove_noise(self,line_measure):
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
