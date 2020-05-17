"""maze_solver_v1 controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
import time 

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
dsNames = ['ds_front', 'ds_FR',  'ds_right', 'ds_BR', 'ds_back', 'ds_BL', 'ds_left', 'ds_FL']

for i in range(8):
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
    for i in range(8):
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


# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(TIME_STEP) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()
    sensors = getSensors()
    rightSpeed = rightMotor.getVelocity()
    leftSpeed= leftMotor.getVelocity()

    L = sensors[7]
    R = sensors[2]
    

    if R < 10:
        leftMotor.setVelocity(2)
        rightMotor.setVelocity(3)
    elif R > 15:
        leftMotor.setVelocity(3)
        rightMotor.setVelocity(2)
    else: 
        setSpeed(3)


    print("Front: ", sensors[0])
    print("Right: ", R)
    
    
    pass

# Enter here exit cleanup code.
