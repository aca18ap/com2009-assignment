"""our_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot

TIME_STEP = 64
robot = Robot()

ds = []
dsNames = ['ds_front', 'ds_FL', 'ds_left', 'ds_FR', 'ds_right', 'ds_BR', 'ds_BL', 'ds_back']

for i in range(8):
    
    ds.append(robot.getDistanceSensor(dsNames[i]))
    ds[i].enable(TIME_STEP)
    
wheels = []
wheelsNames = ['left_wheel', 'right_wheel']

for i in range(2):
    
    wheels.append(robot.getMotor(wheelsNames[i]))
    wheels[i].setPosition(float('inf'))
    wheels[i].setVelocity(0.0)
    
f_ObstacleCounter = 0
fl_ObstacleCounter = 0
fr_ObstacleCounter = 0
turn_counter = 0
turnR = True
move_counter = 0
adjust = 0

while robot.step(TIME_STEP) != -1:
    
    leftSpeed = 7
    rightSpeed = 7
    
    if adjust > 0:
        adjust -= 1
        leftSpeed = -3.4
        rightSpeed = 3.4
    
    elif f_ObstacleCounter > 0 and turnR:
        f_ObstacleCounter -= 1
        leftSpeed = 5.0
        rightSpeed = -5.0
        turn_counter += 1
        if turn_counter == 10:
           turnR = False
           turn_counter = 0
           
    elif f_ObstacleCounter > 0 and not turnR:
        f_ObstacleCounter -= 1
        leftSpeed = -5.0
        rightSpeed = 5.0
        turn_counter += 1
        if turn_counter == 10:
           turnR = True
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
        if ds[0].getValue() < 950.0:
            f_ObstacleCounter = 7
        elif ds[1].getValue() < 950.0:
            fl_ObstacleCounter = 7 
        elif ds[3].getValue() < 950.0:
            fr_ObstacleCounter = 7  
        else:
            move_counter += 1
            if move_counter == 50:
                adjust = 4
                move_counter = 0     
                
    wheels[0].setVelocity(leftSpeed)
    wheels[1].setVelocity(rightSpeed)
        
# Enter here exit cleanup code.
