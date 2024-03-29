"""first_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
from controller import DifferentialWheels

# create the Robot instance.
#robot = Robot()
robot = DifferentialWheels()

# get the time step of the current world.
#timestep = int(robot.getBasicTimeStep())
timestep = 500
#leftMotor = robot.getMotor('left wheel')
#rightMotor = robot.getMotor('right wheel')

#leftMotor.setPosition(float('inf'))
#rightMotor.setPosition(float('inf'))




# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getMotor('motorname')
#  ds = robot.getDistanceSensor('dsname')
#  ds.enable(timestep)

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

    # Process sensor data here.

    #leftMotor.setVelocity(0.5)
    #rightMotor.setVelocity(0.5)
    robot.setSpeed(0.5, 0.5)

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    pass

# Enter here exit cleanup code.
