"""golabi controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
import math
import numpy as np
from controller import *
from sympy import symbols, Eq, solve

#constant values
Delta_t = 0.1  
Deg2Rad = math.pi/180

# get the time step of the current world.
TIME_STEP = 64

# Just used in inverse calculations
def wheels_velocity(matrix, r, two_l): 
  phi1_dot, phi2_dot = symbols('phi1_dot phi2_dot')
  eq1 = Eq(matrix[0], (r/2)*(phi1_dot+phi2_dot))
  eq2 = Eq(matrix[2]*Deg2Rad, (r/two_l)*(phi1_dot-phi2_dot))
  sol = solve((eq1, eq2),(phi1_dot, phi2_dot))
  return sol # phi1_dot and phi2_dot are in radian/s
  
def change_frame(zita_dot, teta, inertialFrame): 
  if inertialFrame:           # changing the frame from robot to inertial
    inertialFrame = -1        # R(-teta)
  else:                       # changing the frame from inertial to robot
    inertialFrame = 1         # R(teta)
  teta *= Deg2Rad
  # rotational matrix
  R_matrix = np.array([
    [math.cos(inertialFrame * teta), math.sin(inertialFrame * teta), 0],
    [-1 * math.sin(inertialFrame * teta), math.cos(inertialFrame * teta), 0 ],
    [0, 0, 1]
  ])
  return np.dot(R_matrix, zita_dot)
  
def inverse_kinematic(matrix, teta, r, two_l):
  return wheels_velocity(change_frame(matrix, teta, False), r, two_l)
  
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
    velocity = inverse_kinematic([10, 10, 0], 45, 20.5, 52)
    phi1_dot, phi2_dot = symbols('phi1_dot phi2_dot')
    left_motor.setVelocity(float(velocity[phi1_dot]))
    right_motor.setVelocity(float(velocity[phi2_dot]))
    
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