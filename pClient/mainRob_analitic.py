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
        global no_driving
        no_driving=0
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

        #the last 4  values of movements
        if len(movements)>3:
            
            movement=movements[-2:]
            if movement==['backward',"forward"] or movements==["forward","backward"]:
                
                if movements[-3]=="left":
                    print("in a loop, turning left")
                    self.driveMotors(-0.1,0.1)
                    movements.append("left")
                else:
                    print("in a loop, turning right")
                    self.driveMotors(0.1,-0.1)
                    movements.append("right")




        #left movements
        if all(line_measure[0:3]):
            self.movement.append("left")
            print("turning left ")
            self.driveMotors(-0.05, 0.05)
            movements.append("left")
        elif all(line_measure[0:2]):
            self.movement.append("left")
            print("turning left -sharp")
            self.driveMotors(-0.1, 0.1)
            movements.append("left")
        elif all(line_measure[1:3]):
            self.movement.append("left")
            print("turning  left -slow  ")
            self.driveMotors(-0.05, 0)
            movements.append("left")

        # forward movements
        elif all(line_measure[3:6]) or all(line_measure[2:5]) or all (line_measure[1:4]) or all(line_measure[1:6]):
            print("moving forward ")
            self.driveMotors(0.15, 0.15)
            movements.append("forward")


        # right movements
        elif all(line_measure[4:7]):
            print("turning right ")
            self.driveMotors(0.05, -0.05)
            movements.append("right")
        elif all(line_measure[5:7]):
            print("turning right - sharp  ")
            self.driveMotors(0.1, -0.1)
            movements.append("right")
        elif all(line_measure[4:6]):
            print("turning right - slow ")
            self.driveMotors(0, -0.05)
            movements.append("right")
        # backward movements
        elif not any(line_measure):
            print("moving backward ")
            self.driveMotors(-0.13, -0.13)
            movements.append("backward")
        else:
            # if any other case, moves forward slowly
            print("moving forward - slow ")
            self.driveMotors(0.15,0.15)
            movements.append("nodriving")


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
