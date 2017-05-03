#import
from i2cMessage import move
import time
import json
import math
import pdb

tol = 1e-5

#defining a point to point function for one neck motor
#part : which servo is being moved
#start : the start position at which the servo should begin at
#finish : the end position of the servo
#speed : the speed at which the servo should move (relates to no current measurement)
def ptpn1(part, start, finish, speed):
   #moving the servo to the start position
    move("neck", part, start)
    #creating limits for the speed of the motor    
    if speed > 10:
        #highest speed is 10
        speed = 10
    elif speed < 0.1:
        #lowest speed is 0.1
        speed = 0.1
    #since the servo is currently at the start position, saying the current position is equal to the start	
    current = start
    #These set if statements find whether servos are being moved from start to finish positions or finish to start
    if start > finish:
        #While the current position is yet to reach the finish limit
	#The sign should match the if statement
        while current > finish:
	    #moving the servo to the current position
	    move("neck", part, current)
	    #decreasing current by the speed multiplied times a constant
	    #Decrease if greater than and increase if less than
	    current -= 0.001*speed
    elif start < finish:
        while current < finish:
            move("neck", part, current)
	    current += 0.001*speed

def ptpn2(part1, start1, finish1, part2, start2, finish2, speed):
    move("neck", part1, start1)
    move("neck", part2, start2)
    if speed > 10:
    	speed = 10
    elif speed < 0.1:
    	speed = 0.1
    current1 = start1
    current2 = start2
    if start1 > finish1:
    	motorDir1 = -1
    else:
    	motorDir1 = 1

    if start2 > finish2:
    	motorDir2 = -1
    else:
    	motorDir2 = 1

    stopAllMotors = 0

    while stopAllMotors == 0:
    	if abs(start1 - finish1) > abs(start1 - current1):
	    move("neck", part1, current1)
	    current1 += motorDir1*0.001*speed
	if abs(start2 - finish2) > abs(start2 - current2):
	    move("neck", part2, current2)
	    current2 += motorDir2*0.001*speed
	if current1 >= finish1 - tol and current2 >= finish2 - tol:
	    stopAllMotors = 1

#The same as previous
#This function moves 3 servos simultaneously.
def ptpn3(part1, start1, finish1, part2, start2, finish2, part3, start3, finish3, speed):
    #Setting initial positions
    move("neck", part1, start1)
    move("neck", part2, start2)
    move("neck", part3, start3)
    #Limiting speed
    if speed > 10:
        speed = 10
    elif speed < 0.1:
        speed = 0.1
    #Initializing current values
    current1 = start1
    current2 = start2
    current3 = start3
    #Determining whether to increase or decrease the servo position
    if start1 > finish1:
    	motorDir1 = -1
    else:
    	motorDir1 = 1 

    if start2 > finish2:
    	motorDir2 = -1
    else:
    	motorDir2 = 1 

    if start3 > finish3:
    	motorDir3 = -1
    else:
    	motorDir3 = 1 

    stopAllMotors = 0

    while stopAllMotors == 0:
	if abs(start1 - finish1) > abs(start1 - current1): 
	    move("neck", part1, current1)
	    current1 += motorDir1*0.001*speed
	if abs(start2 - finish2) > abs(start2 - current2): 
	    move("neck", part2, current2)
	    current2 += motorDir2*0.001*speed
	if abs(start3 - finish3) > abs(start3 - current3): 
	    move("neck", part3, current3)
	    current3 += motorDir3*0.001*speed
	if current1 >= finish1 - tol and current2 >= finish2 - tol and current3 >= finish3 - tol:
            stopAllMotors = 1

#Function for setting neck servos to their default positions
def defaultNeckPos():
    move("neck", "tilt", 0.8)
    move("neck", "roll", 0.5)
    move("neck", "pan", 0.5)
