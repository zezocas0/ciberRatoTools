---
marp: true
theme: gaia
_class: lead
paginate: true
backgroundColor: #fff
backgroundImage: url('https://marp.app/assets/hero-background.svg')
---
# Assignment 2
Robotic challenge solver
using the CiberRato simulation environment

José Santos 98279

--- 

# Introduction
The objective of this assignment is to develop a robotic challenge solver using the CiberRato simulation environment. The challenge consists of a path with several possible solutions and the robot must map, explore and find the best solution in the given time.

---
# objectives
- Localization 
    - use of the line sensor to detect the lines of the path and the ground sensor to detect the beacons and a motion model to estimat the positoion of the robot.

- Mapping
    - use of the logs to map the path and the possible solutions to create a map that has all beacons that spawned randomly.

- Planning
  - Use of the created map to plan the best path passing through all beacons and returning to the initial position

---
# Functions used
In this assignment, I implemented several functions to deal with the objectives given, however I wasnt able to implement correctly all of them. 


---
# Localization

- movement model 
    - Given the 4 possible actions, moving forward,left,right and backwards, for each we need to calculate the position of the robot. For that we use the movement model,shown below
- movement model code
```python
    lin= (motorL+motorR)/2*noise 
    rot= (motorR-motorL)/1 
    t_last=self.estdir
    x=self.estx+lin*cos(t_last)
    y=self.esty+lin*sin(t_last)
```
---
# Movement 
- Besides having the movement model, For the robot to mainain its orientation on the path, while going on a straight line I implemented a PID controller, that would keep the robot on the path, however I wasnt able to implement it correctly. For the rotations the robot uses the orientation `theta` from the movement model to rotate the robot to the desired orientation. 

---
# Pathing 
- For the robot to move around the map, I implemented a function that according to the linesensor and the robot orientation would determine which one of the for actions to take, saving it in a logging dictionary.
---
# Logging
  - The logging dictionary has the crossroads as keys and the values are a list with the possible actions, the actions that have been done and the the ground detection result .
  - The actions to be done are saved accodring to the initial orientation of the robot, so that the variables are updated the same way for all the crossroads.
  - The actions saved were calculated on another function, this being the `determine_action()` function.


---
# Mapping
   
- from the `logging` function , we obtain the beacons and the crossroads from the possible movements, mapping the path as the robot moves.
- The implementation of creating the map was based on the previously assigned possible movements at each crossoroads, and the beacons detected at the crossroads. 
- With these we could figure out whether we are able to move forward ,simbolized as '-' in the map or left or right, simbolized as  '|', related to each crossroad found. 
However due to other errors, the map wasnt created correctly. 

---
# Pathing
Because of the use of the completed map to plan the path, the pathing function wasnt implemented, however the idea was to use the map connections to each crossroad,which would be replaced with '+' in the map. To plan the best path, I was going to either implement a Greedy algorithm due to the small size of the map, being able to compute all possible paths in a short ammount of time.
A* algorithm would be also be a option to implement, however, i wasnt able to experiment with each of them.

---
# run.sh
- The run.sh file was created to run the main simulation file with the parameters given by the user, these being:
  - c : type of map, in this case the C4 maps
  - p : initial position of the robot, as it can be placed on any beacon
  - h : server connection
  - r : robot name
  - f ; file name for the map
---
## difficulties and issues 


- Due to the complexity of the assignment, and not having a specific order or idea on how to implement the methods, I had some difficulties in implementing the functions, having to redo several functions more than once.
- Another issue was the lack of time to implement the functions correctly, as I wasnt able to implement the mapping and pathing function  correctly, which was the most important function to implement the pathing function.
   
---

# conclusion
- In conclusion, I wasnt able to implement the assignment correctly, however I was able to learn a lot about the CiberRato simulation environment and the use of the python language, which I had never used before.


---