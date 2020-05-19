"""our_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot

TIME_STEP = 64
robot = Robot()

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
    
    while robot.step(TIME_STEP) != -1:
        
        leftSpeed = 13
        rightSpeed = 13
                 
        
        if init == True:
            if duration1 > 0:
                duration1 -= 1   
                leftSpeed = 5.0
                rightSpeed = -5.0
            elif duration2 > 0:
                duration2 -= 1   
                leftSpeed = -5.0
                rightSpeed = 5.0
                
            else :
                init = False   
        
        elif adjust > 0:
            adjust -= 1
            leftSpeed = -3.4
            rightSpeed = 3.4
        
        elif f_ObstacleCounter > 0 and turnL:
            f_ObstacleCounter -= 1
            leftSpeed = -5.0
            rightSpeed = 5.0
            turn_counter += 1
            if turn_counter == 10:
               turnL = False
               turn_counter = 0
               
        elif f_ObstacleCounter > 0 and not turnL:
            f_ObstacleCounter -= 1
            leftSpeed = 5.0
            rightSpeed = -5.0
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
    

        
# Enter here exit cleanup code.
