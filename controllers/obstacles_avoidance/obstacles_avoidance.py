"""our_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
from controller import Camera

TIME_STEP = 64
robot = Robot()


# set up camera
camera = robot.getCamera('camera')
camera.enable(TIME_STEP) 

sampleSize = 5 # define size of the central pixel grid


"""
Given average r, g and b values from a sample, 
determines which of the colours it represents.
"""    
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
    elif (r>100): 
        if (b>100):
            col = "PURPLE"
        elif (g>100):
            col = "OLIVE"
        else:
            col = "MAROON"
    elif (b>100):
        if (g>100):
            col = "TEAL"
        else:
            col = "NAVY"
    elif (g>100):
        col = "GREEN"
    return col






"""
Takes the average rgb values of the central pixels from the camera.
"""
def check_central_colour():

    pic = camera.getImageArray()
    mid_w = camera.getWidth()/2
    mid_h = camera.getHeight()/2
    r = 0
    g = 0
    b = 0
    
    # sum the colour components for the central 100 pixels (10x10)
    for x in range(int(mid_w-sampleSize/2),int(mid_w+sampleSize/2)):
      for y in range(int(mid_h-sampleSize/2),int(mid_h+sampleSize/2)):
        r += pic[x][y][0]
        g += pic[x][y][1]
        b += pic[x][y][2]
    print ('r' + str(r/(sampleSize*sampleSize)) +' g' + str(g/(sampleSize*sampleSize)) +' b' + str(b/(sampleSize*sampleSize)))
    
    col = classify_colour(r/(sampleSize*sampleSize),g/(sampleSize*sampleSize),b/(sampleSize*sampleSize))
    print(col)
    return(col)






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
    colorToFind = ""
    turn_around = 0
    adjust_acount = 0
    aTurnL = True
    
    
    while robot.step(TIME_STEP) != -1:
        
        leftSpeed = 9.5
        rightSpeed = 9.5
                 
        currentColor = check_central_colour()
        
        
        ## Initial turn to detect box color
        if init == True:
            if duration1 > 0:
                duration1 -= 1   
                leftSpeed = 5.0
                rightSpeed = -5.0
                colorToFind = check_central_colour()
            elif duration2 > 0:
                duration2 -= 1   
                leftSpeed = -5.0
                rightSpeed = 5.0
            else :
                init = False   
        
        elif (currentColor == colorToFind) and (ds[0].getValue() < 10):         
            wheels[0].setVelocity(0)
            wheels[1].setVelocity(0)
            print("target found")
            break
 
        elif turn_around > 0:
            turn_around -= 1
            leftSpeed = -4.0
            rightSpeed = 4.0    
        
        elif adjust > 0 and aTurnL:
            adjust -= 1
            leftSpeed = -2.9
            rightSpeed = 2.9 
            if adjust_acount == 7 and adjust ==0:
              aTurnL = False
              adjust_acount = 0
              
        elif adjust > 0 and not aTurnL:
            adjust -= 1
            leftSpeed = 2.9
            rightSpeed = -2.9 
            if adjust_acount == 7 and adjust ==0:
              aTurnL = True
              adjust_acount = 0     
        
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
            if (ds[2].getValue() < 20 and ds[6].getValue() < 20) or (ds[1].getValue() < 20 and ds[5].getValue() < 20) or (ds[3].getValue() < 20 and ds[7].getValue() < 20):
                print("turn_around")
                turn_around = 17.5          
            elif ds[0].getValue() < 20:
                f_ObstacleCounter = 4.2
            elif ds[3].getValue() < 20 or ds[1].getValue() < 20 or ds[2].getValue() < 20 or ds[4].getValue() < 20:
                fl_ObstacleCounter = 4.2 
            elif ds[7].getValue() < 20 or ds[5].getValue() < 20 or ds[6].getValue() < 20 or ds[8].getValue() < 20:
                fr_ObstacleCounter = 4.2  
            else:
                move_counter += 1
                if move_counter == 9:
                    adjust = 2
                    adjust_acount += 1
                    move_counter = 0     
                    
        wheels[0].setVelocity(leftSpeed)
        wheels[1].setVelocity(rightSpeed)

   
obs_avoidance()