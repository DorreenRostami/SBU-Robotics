"""camera_controller controller."""

from controller import Robot, Camera

robot = Robot()

timestep = 32

cam = robot.getDevice('camera')
cam.enable(timestep)

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    success = cam.saveImage("img.jpg", 100)
    pass

# Enter here exit cleanup code.
