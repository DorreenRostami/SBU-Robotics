"""sonar_controller controller."""

from controller import Robot, DistanceSensor
import math

timestep = 32

robot = Robot()

# get the motor devices
left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')
# set the target position of the motors
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(-0.01)
right_motor.setVelocity(0.01)

#get and enable distance sensors (sonars)
dsNames = ['ds0', 'ds1', 'ds2', 'ds3','ds4', 'ds5', 'ds6', 'ds7']    
ds = []
for i in range(8):
    ds.append(robot.getDevice(dsNames[i]))
    ds[i].enable(timestep)
 
values = [] #the array that we will hold the 256 values inside  
wanted_angle = 0 #the angle of the robot when appending values

#get and enable compass
compass = robot.getDevice('compass')
compass.enable(timestep) 

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    north = compass.getValues()
    angle = math.atan2(north[2], north[0]) #atan(z/x)
    angle = angle - 0.00022  
    if angle < 0: #if angle < 0
        angle = angle + 6.28 #angle + 2pi
        
    if angle >= wanted_angle:
        for i in range(8):
            values.append(ds[i].getValue())  
        wanted_angle = wanted_angle + 0.19625
    
        if len(values) == 256:
            left_motor.setVelocity(0.0)
            right_motor.setVelocity(0.0)
            print("{}".format(values), "\n")

    pass

# Enter here exit cleanup code.
