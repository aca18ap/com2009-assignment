"""maze_solver_v1 controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
import time 
import math

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
TIME_STEP = int(robot.getBasicTimeStep())
MAX_SPEED = 6.28
# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getMotor('motorname')
#  ds = robot.getDistanceSensor('dsname')
#  ds.enable(timestep)

ds = []
dsNames = ['ds_front', 'ds_FL15', 'ds_FL30', 'ds_FL45', 'ds_FL60', 'ds_FR15', 'ds_FR30', 'ds_FR45', 'ds_FR60', 'ds_BR60', 'ds_BL60', 'ds_back']

for i in range(12):
    
    ds.append(robot.getDistanceSensor(dsNames[i]))
    ds[i].enable(TIME_STEP)
    
wheels = []
wheelsNames = ['left_wheel', 'right_wheel']

for i in range(2):
    
    wheels.append(robot.getMotor(wheelsNames[i]))
    wheels[i].setPosition(float('inf'))
    wheels[i].setVelocity(0.0)

leftMotor = wheels[0]
rightMotor = wheels[1]


## General Functions

def setSpeed(speed):
    leftMotor.setVelocity(speed)
    rightMotor.setVelocity(speed)
    
def halt():
    leftMotor.setVelocity(0)
    rightMotor.setVelocity(0)
    
def turn(deg):
    halt()
    if deg < 180:
        leftMotor.setVelocity(deg * 0.075)
    else:
        rightMotor.setVelocity(deg * 0.75)


def getSensors():
    val = []
    for i in range(12):
        val.append(ds[i].getValue())
    return val
    
def findClosest():
    sensors = getSensors()
    tmp = 0
    for i in range(len(sensors)):
        if sensors[i] < sensors[tmp]:
            tmp = i
    return(i)            


def findFurthest():
    sensors = getSensors()
    tmp = 0
    for i in range(len(sensors)):
        if sensors[i] > sensors[tmp]:
            tmp = i
    return(i)  


cos30 = math.sqrt(3)/2
default = 6
# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(TIME_STEP) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()
    sensors = getSensors()
    rightSpeed = rightMotor.getVelocity()
    leftSpeed= leftMotor.getVelocity()
    
    front = ds[0].getValue()
    R0 = ds[5].getValue()
    R1 = ds[8].getValue() * cos30
    R2 = ds[9].getValue() * cos30
    
    
    
    dist = (R1 + R2)/2
    diff = R1 - R2
    if front < 8 or R0 < 6 or R1 < 6:
        halt()
        print("obstacle ahead")
        leftMotor.setVelocity(-1.5)
        rightMotor.setVelocity(1.5)
    else:
        if dist < 40:
            if dist < 10:
                if dist < 7:
                    print("way too close")
                    leftMotor.setVelocity(-1)
                    rightMotor.setVelocity(1)
                else:
                    print("too close")
                    leftMotor.setVelocity(0)
                    rightMotor.setVelocity(default)
            else:
                if R0 < 25:
                    print("wall right ahead")
                    leftMotor.setVelocity(default * 0.1)
                    rightMotor.setVelocity(default * 1.5)
                elif diff > 6:
                    print("adj right")
                    rightMotor.setVelocity(default * 0.3)
                    leftMotor.setVelocity(default)
                elif diff < -6:
                    print("adj left")
                    rightMotor.setVelocity(default)
                    leftMotor.setVelocity(default * 0.3)
                else:
                    print("perfect")
                    setSpeed(default * 1.5)
        else:
            if R2 < 35:
                print("still turning")
                rightMotor.setVelocity(default * 0.7)
                leftMotor.setVelocity(default * 0.9)    
            else:
                print("too far")
                rightMotor.setVelocity(rightSpeed * 0.6)
                leftMotor.setVelocity(default * 0.8)
            
       
    

    #print("R0= ", R0, "R1= ", R1, " || R2= ", R2, " || dist= ", dist)
    
    
    pass

# Enter here exit cleanup code.
