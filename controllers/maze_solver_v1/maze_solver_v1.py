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


##Variables Declarations
f_ObstacleCounter = 0
fl_ObstacleCounter = 0
fr_ObstacleCounter = 0
turn_counter = 0
turnL = True
move_counter = 0
adjust = 0
init = True

colorToFind = ""
cos30 = math.sqrt(3)/2
default = 6
beacon_part = False
beacon_found = False
turn_around = 0
turnL_a = True
turn_counter_a = 0


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


camera = robot.getCamera('camera')
camera.enable(TIME_STEP) 

camera_floor = robot.getCamera('camera_floor')
camera_floor.enable(TIME_STEP)

sampleSize = 5 # define size of the central pixel grid



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





def navigate_maze():
    rightSpeed = rightMotor.getVelocity()
    leftSpeed= leftMotor.getVelocity()
    
    front = ds[0].getValue()
    R0 = ds[5].getValue()
    R1 = ds[8].getValue() * cos30
    R2 = ds[9].getValue() * cos30
    
    
    
    dist = (R1 + R2)/2
    diff = R1 - R2
        
    
    if front < 8 or R0 < 7 or ds[8].getValue() < 8 or ds[2].getValue() < 15:
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
            else :
                print("too far")
                rightMotor.setVelocity(rightSpeed * 0.6)
                leftMotor.setVelocity(default * 0.8)
                
        
                
"""
Given average r, g and b values from a sample, 
determines which of the colours it represents.
"""    
def classify_colour(r,g,b):

    col = "UNKNOWN"
    
    #first check if we see a floor!
    if (r >200 and b>200 and g>200):
        return "WHITE-FLOOR"
    
    #now check if we see a wood panel
    
    #see a wall
    if ( (r-b) < 5 and (r-b) > -5 and (g-b) < 5 and (g-b) >-5 ):
        return "GREY"
    
    #see a wood panel
    if (r>200 and g>180 and b>120):
        return "WOOD"
    

    # not floor, wall, or wood panel. determine the beacon colour
    if (r>200):
        if (b>200):
            col = "FUSCHIA"
        elif (g>200):
            col = "YELLOW"
        else:
            col = "RED"
    elif (b>200):
        if (g>200):
            col = "AQUA"
        else:
            col = "BLUE"
    elif (g>200):   
        col = "LIME"
    elif (r>120): 
        if (b>120):
            col = "PURPLE"
        elif (g>120 and b<100):
            col = "OLIVE"
        else:
            col = "MAROON"
    elif (b>120):
        if (g>120):
            col = "TEAL"
        else:
            col = "NAVY"
    elif (g>120):
        col = "GREEN"
    return col






"""
Takes the average rgb values of the central pixels from the camera.
"""
def check_central_colour(cameraObject):

    pic = cameraObject.getImageArray()
    mid_w = cameraObject.getWidth()/2
    mid_h = cameraObject.getHeight()/2
    r = 0
    g = 0
    b = 0
    
    # sum the colour components for the central 100 pixels (10x10)
    for x in range(int(mid_w-sampleSize/2),int(mid_w+sampleSize/2)):
      for y in range(int(mid_h-sampleSize/2),int(mid_h+sampleSize/2)):
        r += pic[x][y][0]
        g += pic[x][y][1]
        b += pic[x][y][2]
    #print ('r' + str(r/(sampleSize*sampleSize)) +' g' + str(g/(sampleSize*sampleSize)) +' b' + str(b/(sampleSize*sampleSize)))
    
    col = classify_colour(r/(sampleSize*sampleSize),g/(sampleSize*sampleSize),b/(sampleSize*sampleSize))
    #print(col)
    return(col)                
 
 
 
def check_initial_color():
    duration1 = 10
    duration2 = 10
    init = True
    while init == True:
        print(duration1, duration2)
        if duration1 > 0:
            duration1 -= 1   
            leftMotor.setVelocity(5)
            rightMotor.setVelocity(-5)
            colorToFind = check_central_colour(camera)
        elif duration2 > 0:
            duration2 -= 1   
            leftMotor.setVelocity(-5)
            rightMotor.setVelocity(5)
        else:
            init = False
 


def beacon_finder(adjust, f_ObstacleCounter, fl_ObstacleCounter, fr_ObstacleCounter, turn_counter, move_counter, turnL, turn_around, turnL_a, turn_counter_a):
    current_color = check_central_colour(camera)
    leftSpeed = 9;
    rightSpeed = 9;
    global beacon_found
    if (current_color == colorToFind) and (ds[0].getValue() < 15):         
            wheels[0].setVelocity(0)
            wheels[1].setVelocity(0)
            beacon_found = True
            print("target reached")
  
            
    elif turn_around > 0 and turnL_a:
            turn_around -= 1
            leftSpeed = -4.1
            rightSpeed = 4.1
            turn_counter_a += 1
            if turn_counter == 16:
               turnL_a = False
               turn_counter_a = 0
               
    elif turn_around > 0 and not turnL_a:
        turn_around -= 1
        leftSpeed = 4.1
        rightSpeed = -4.1
        turn_counter_a += 1
        if turn_counter == 16:
           turnL_a = True
           turn_counter_a = 0   


    elif f_ObstacleCounter > 0 and turnL:
        f_ObstacleCounter -= 1
        leftSpeed = -3.7
        rightSpeed = 3.7
        print("turn_left_F")
        turn_counter += 1
        if turn_counter == 5:
           turnL = False
           turn_counter = 0
               
    elif f_ObstacleCounter > 0 and not turnL:
        f_ObstacleCounter -= 1
        leftSpeed = 3.7
        rightSpeed = -3.7
        print("turn_right_F")
        turn_counter += 1
        if turn_counter == 5:
           turnL = True
           turn_counter = 0    

    elif fl_ObstacleCounter > 0:
        fl_ObstacleCounter -= 1
        leftSpeed = 5.0
        rightSpeed = -5.0
        print("turn_right")
        
    elif fr_ObstacleCounter > 0:
        fr_ObstacleCounter -= 1
        leftSpeed = -5.0
        rightSpeed = 5.0
        print("turn_left")
        
    elif adjust > 0:
        adjust -= 1
        leftSpeed = 5
        rightSpeed = -5    
        
    else:  # read sensors
            if (ds[4].getValue() < 15 and ds[8].getValue() < 15):
                print("turn_around")
                turn_around = 16.0          
            elif ds[0].getValue() < 15:
                f_ObstacleCounter = 5.0
            elif ds[3].getValue() < 15 or ds[1].getValue() < 15 or ds[2].getValue() < 15 or ds[4].getValue() < 15:
                fl_ObstacleCounter = 5.5 
            elif ds[7].getValue() < 15 or ds[5].getValue() < 15 or ds[6].getValue() < 15 or ds[8].getValue() < 15:
                fr_ObstacleCounter = 5.5 
            else:
                move_counter += 1
                if move_counter == 50:
                    scan_around()
                elif move_counter == 76:
                    adjust = 4
                    move_counter = 0    
                
    wheels[0].setVelocity(leftSpeed)
    wheels[1].setVelocity(rightSpeed)
    return((adjust, f_ObstacleCounter, fl_ObstacleCounter, fr_ObstacleCounter, turn_counter, move_counter, turnL, turn_around, turnL_a, turn_counter_a))



def scan_around():
    duration = 100
    if duration > 0:
        duration -= 1
        leftMotor.setVelocity(5)
        rightMotor.setVelocity(-5)
        if check_central_colour(camera) == colorToFind:
            halt()
            return True
    else:
        halt()
        return False
    
def find_beacon():
    found = scan_around()

    if found == True:
        setSpeed(4)
    else:
        if ds[0].getValue() > 40:
            setSpeed(4)      
        elif ds[0].getValue() < 15:
            found = scan_around()
            leftMotor.setVelocity(5)
            rightMotor.setVelocity(-5)
    
        

# Main loop:
# - perform simulation steps until Webots is stopping the controller




initialCounter = 0
duration2 = 50
duration1 = 50
print(TIME_STEP)
bcnRes = (adjust, f_ObstacleCounter, fl_ObstacleCounter, fr_ObstacleCounter, turn_counter, move_counter, turnL, turn_around, turnL_a, turn_counter_a)
while robot.step(TIME_STEP) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()
    
    if initialCounter != 1:
        print("initial color check")
        
        if duration1 > 0:
            duration1 -= 1   
            leftMotor.setVelocity(5)
            rightMotor.setVelocity(-5)
            colorToFind = check_central_colour(camera)
        elif duration2 > 0:
            duration2 -= 1   
            leftMotor.setVelocity(-5)
            rightMotor.setVelocity(5)
        else:
            initialCounter +=1
            print("Color to find: ",colorToFind)
            
        
    else:
        current_floor = check_central_colour(camera_floor)
        print(beacon_found)
        print(current_floor)
        if (current_floor != "WHITE-FLOOR" and current_floor != "GREY") and beacon_part == False:
            beacon_part = True
            print("Switched to beaconing challenge mode")
                  
        if beacon_part == False:
            print("current floor camera color: ",current_floor)
            navigate_maze()
        elif beacon_part == True and beacon_found != True:
            #find_beacon()
            print("current beacon ahead: ", check_central_colour(camera))
            bcnRes = beacon_finder(bcnRes[0], bcnRes[1], bcnRes[2], bcnRes[3], bcnRes[4], bcnRes[5], bcnRes[6], bcnRes[7], bcnRes[8], bcnRes[9] ) 
        elif beacon_found == True:
            halt()
            break
        
    
    pass

# Enter here exit cleanup code.


