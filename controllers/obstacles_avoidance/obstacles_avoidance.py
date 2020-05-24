"""our_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
from controller import Camera

TIME_STEP = 64
robot = Robot()


camera = robot.getCamera('camera')
camera.enable(TIME_STEP) #you can tell this works because the camera turns on in the simulation!

def classify_colour(r,g,b):
    col = "UNKNOWN"
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
    elif (r>64): 
        if (b>64):
            col = "PURPLE"
        elif (g>64):
            col = "OLIVE"
        else:
            col = "MAROON"
    elif (b>64):
        if (g>64):
            col = "TEAL"
        else:
            col = "NAVY"
    elif (g>64):
        col = "GREEN"
    return col


def check_central_colour():

    pic = camera.getImageArray()
    picAsArray = camera.getImage()
    mid_w = camera.getWidth()/2
    mid_h = camera.getHeight()/2
    r = 0
    g = 0
    b = 0
    sampleSize = 10 #take the central 10x10 pixel grid
        
    # sum the colour components for the central 100 pixels (10x10)
    for x in range(int(mid_w-sampleSize/2),int(mid_w+sampleSize/2)):
      for y in range(int(mid_h-sampleSize/2),int(mid_h+sampleSize/2)):
        r += pic[x][y][0]
        g += pic[x][y][1]
        b += pic[x][y][2]
    
    col = classify_colour(r/sampleSize,g/sampleSize,b/sampleSize)
    
    return col

"""
Given average r, g and b values from a sample, 
determines which of the colours it represents.
"""    






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
                   

def obs_avoidance():

    f_ObstacleCounter = 0
    fl_ObstacleCounter = 0
    fr_ObstacleCounter = 0
    turn_counter = 0
    turnL = True
    move_counter = 0
    adjust = 0
    init = True
    duration1 = 10
    duration2 = 10
    col = ""
    
    while robot.step(TIME_STEP) != -1:
        
        leftSpeed = 10
        rightSpeed = 10
        
        col = check_central_colour
        print(col)
                        
        if init == True:
            if duration1 > 0:
                duration1 -= 1   
                leftSpeed = 5.0
                rightSpeed = -5.0            
            elif duration2 > 0:
                duration2 -= 1   
                leftSpeed = -5.0
                rightSpeed = 5.0               
            else:
                init = False   
        
        elif adjust > 0:
            adjust -= 1
            leftSpeed = -2.8
            rightSpeed = 2.8
        
        elif f_ObstacleCounter > 0 and turnL:
            f_ObstacleCounter -= 1
            leftSpeed = -4.0
            rightSpeed = 4.0
            turn_counter += 1
            if turn_counter == 10:
               turnL = False
               turn_counter = 0
               
        elif f_ObstacleCounter > 0 and not turnL:
            f_ObstacleCounter -= 1
            leftSpeed = 4.0
            rightSpeed = -4.0
            turn_counter += 1
            if turn_counter == 10:
               turnL = True
               turn_counter = 0    
    
        elif fl_ObstacleCounter > 0:
            fl_ObstacleCounter -= 1
            leftSpeed = 5.0
            rightSpeed = -5.0
            
        elif fr_ObstacleCounter > 0:
            fr_ObstacleCounter -= 1
            leftSpeed = -5.0
            rightSpeed = 5.0
            
        else:  # read sensors
            if ds[0].getValue() < 10:
                f_ObstacleCounter = 7
            elif ds[3].getValue() < 10 or ds[1].getValue() < 10 or ds[2].getValue() < 10 or ds[4].getValue() < 10:
                fl_ObstacleCounter = 7 
            elif ds[7].getValue() < 10 or ds[5].getValue() < 10 or ds[6].getValue() < 10 or ds[8].getValue() < 10:
                fr_ObstacleCounter = 7  
            else:
                move_counter += 1
                if move_counter == 50:
                    adjust = 4
                    move_counter = 0     
                    
        wheels[0].setVelocity(leftSpeed)
        wheels[1].setVelocity(rightSpeed)

   
obs_avoidance()
    
    

