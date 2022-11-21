"""golabi controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor

from controller import *

# get the time step of the current world.
TIME_STEP = 64


# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)

# Main loop:
# - perform simulation steps until Webots is stopping the controller

def init_robot(robot, gps):
    # get the motor devices
    left_motor = robot.getDevice('left wheel motor')
    right_motor = robot.getDevice('right wheel motor')
    # set the target position of the motors
    left_motor.setPosition(float('inf'))
    right_motor.setPosition(float('inf'))

    ps = []
    psNames = [
        'ps0', 'ps1', 'ps2', 'ps3',
        'ps4', 'ps5', 'ps6', 'ps7'
    ]
    
    for i in range(8):
        ps.append(robot.getDevice(psNames[i]))
        ps[i].enable(TIME_STEP)
    #changing wheels' velocity
    left_motor.setVelocity(1)
    right_motor.setVelocity(5)
    
def init_gps(gps):
    gps.enable(TIME_STEP)
    
    
def main():
    my_robot = Robot()
    my_gps = GPS('gps')
    init_gps(my_gps)
    init_robot(my_robot, my_gps)
    
    while my_robot.step(TIME_STEP) != -1:
        # Read the sensors:
        # Enter here functions to read sensor data, like:
        #  val = ds.getValue()
        
        res = my_gps.getValues()
        #print(res[0], res[1], res[2])
        # Process sensor data here.

        # Enter here functions to send actuator commands, like:
        #  motor.setPosition(10.0)
        pass


if __name__ == "__main__":
    main()