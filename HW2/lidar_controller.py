"""lidar controller."""

from controller import Robot, Lidar

robot = Robot()

timestep = 288

 
#Lidar
lidar = robot.getDevice('lidar')
lidar.enable(timestep)
lidar.enablePointCloud()


# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    values = lidar.getRangeImage()
    print("{}".format(values[0:90]))
    pass

# Enter here exit cleanup code.
