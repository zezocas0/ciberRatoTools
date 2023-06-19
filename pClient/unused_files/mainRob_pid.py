import sys
from croblink import *
from math import *
import xml.etree.ElementTree as ET
from datetime import datetime

CELLROWS=7
CELLCOLS=14



class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.error_sum = 0
        self.last_error = 0
        self.last_time = datetime.now()

    def update(self, error):
        # Calculate the time step as the difference between the current time and the last time
        time_now = datetime.now()
        dt = (time_now - self.last_time).total_seconds()
        self.last_time = time_now


        self.error_sum += error * dt
        derivative = (error - self.last_error) / dt
        self.last_error = error
        return self.kp * error + self.ki * self.error_sum + self.kd * derivative



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
        # Set the PID constants
        kp= 0.2
        ki =0.0499
        kd =0.000
        #calling the class PIDController
        pid_controller = PIDController(kp, ki, kd)
        # Preprocess the line sensor measurements to remove noise 
        line_measure = remove_noise(self,self.measures.lineSensor)
        #if it is not on the line, it goes backwards 
        if not any(line_measure):
            self.driveMotors(-0.1,-0.1)
        else:
            error = 0.0
            for i in range(len(line_measure)):
                error += (i - 2.5) * line_measure[i]
            #try catch cause sum(line_measure) can be 0
            try:
                error /= sum(line_measure)
            except:
                error=0.0
            # Update the PID controller with the error, time step calculated inside the pid_controller.update 
            control_output = pid_controller.update(error)
            # drive motors using control_output

            self.driveMotors(0.15+control_output,0.15-control_output)




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
