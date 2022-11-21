# important points:
# use <robot_position> to get current position of robot in <x,y,theta> format.
# use <robot_omega> to get current values for the wheels in <w1,w2,w3> format.

import numpy as np
import math
from initialization import * 

goal_position = np.array([1.3,6.15]) #x,y
start_position = np.array([1.3,-9.74]) #x,y
line_slope = 0
goal_robot_distance = 0
saved_point = np.array([1.3,-9.74])


#compass angles when looking left/up/right/dowm
LOOK_DOWN = -1.573
LOOK_LEFT = 0.002
LOOK_UP = 1.573
LOOK_RIGHT = 3.134

GO_RIGHT = [1,-2,1]
GO_LEFT = [-1,2,-1]
GO_FWD = [2,0,-2]
GO_BWD = [-2,0,2]
GO_CW = [1,1,1]
GO_CCW = [-1,-1,-1]

def is_facing(angle_robot_north, angle_goal_robot): 
    
    if angle_robot_north < 0: #if angle < 0
        angle_robot_north = angle_robot_north + 6.28 #angle + 2pi
    if angle_goal_robot < 0: #if angle < 0
        angle_goal_robot = angle_goal_robot + 6.28 #angle + 2pi
     
    quarter_goal = -1
    if angle_goal_robot <= 6.28 and angle_goal_robot > 4.70 :
        quarter_goal = 4 
    elif angle_goal_robot >= 0 and angle_goal_robot < 1.58 :
        quarter_goal = 1
        
    quarter_north = -1
    if angle_robot_north <= 6.28 and angle_robot_north > 4.71 :
        quarter_north = 4 
    elif angle_robot_north > 0 and angle_robot_north < 1.57 :
        quarter_north = 1
             
      
    # print("{}".format(angle_robot_north), " north \n")
    # print("{}".format(angle_goal_robot), " goal \n")

    if angle_goal_robot < angle_robot_north + 0.01 and angle_goal_robot > angle_robot_north - 0.01:
        return 1 #is facing
    elif angle_robot_north > angle_goal_robot and not(quarter_north == 4 and quarter_goal == 1) \
        or (quarter_north == 1 and quarter_goal == 4):
        return 0 #turn clockwise
    return -1 #turn counter clockwise

def rotate_to_angle(current_angle, new_angle):
    is_facing_output = is_facing(current_angle, new_angle)
    if current_angle < new_angle + 0.01 and current_angle > new_angle - 0.01:
        return 1 #is facing
    elif current_angle > new_angle:
        update_motor_speed(GO_CW)
        return 0 #turn counter clockwise
    update_motor_speed(GO_CCW)
    return 0 #turn clockwise

seen_obs_state = 5
steps_taken = 0
goal_angle = 0
rob_ang_prev = 0
starting_turn = 1
finished_turn = 0
sensor2_before_turn = -1
sensor5_before_turn = -1
flag = 0

#check if there is an obstcle in the way
#if we're going right, turn to the right and check
#else turn to the left and check
def check_obs_in_way(ir_value, rflag):
    global seen_obs_state
    global goal_angle
    global steps_taken
    global rob_ang_prev
    global starting_turn
    global finished_turn
    global sensor2_before_turn
    global sensor5_before_turn
    global flag
    
    if flag == 1: #is done turning and facing the wall
        if ir_value[FIRST_IR] != 1000 and ir_value[SECOND_IR] != 1000: #sees a full wall            
            if abs(ir_value[FIRST_IR] - ir_value[SECOND_IR]) > 200: #one sensor is on a further wall or space
                if ir_value[FIRST_IR] < ir_value[SECOND_IR]: #right wheel is farther than the left 
                    if ir_value[FIRST_IR] > 300:
                        update_motor_speed(GO_FWD)
                    elif ir_value[FIRST_IR] < 200:
                        update_motor_speed(GO_BWD)  
                    else:
                        if rflag:
                            update_motor_speed(GO_RIGHT)
                        else:
                            update_motor_speed(GO_LEFT)
                        steps_taken = 0
                        flag = 0
                else:
                    if ir_value[SECOND_IR] > 300:
                        update_motor_speed(GO_FWD)
                    elif ir_value[SECOND_IR] < 200:
                        update_motor_speed(GO_BWD)  
                    else:
                        if rflag:
                            update_motor_speed(GO_RIGHT)
                        else:
                            update_motor_speed(GO_LEFT) 
                        steps_taken = 0
                        flag = 0
                           
            elif abs(ir_value[FIRST_IR] - ir_value[SECOND_IR]) > 20: 
                if ir_value[FIRST_IR] > ir_value[SECOND_IR]: #right wheel is farther than the left 
                    if rflag:
                        update_motor_speed(GO_CCW)   
                    else:
                        update_motor_speed(GO_CW) 
                else:
                    if rflag:
                        update_motor_speed(GO_CW)   
                    else:
                        update_motor_speed(GO_CCW) 
            elif ir_value[FIRST_IR] > 750 or ir_value[SECOND_IR] > 750:
                update_motor_speed(GO_FWD)
            elif ir_value[FIRST_IR] < 600 or ir_value[SECOND_IR] < 600:
                update_motor_speed(GO_BWD) 
            else:
                if rflag:
                    update_motor_speed(GO_RIGHT)
                else:
                    update_motor_speed(GO_LEFT) 
                steps_taken = 0
                flag = 0
        else: # reaching/has reached end of the wall
            if ir_value[FIRST_IR] == 1000:
                if ir_value[SECOND_IR] > 250:
                    update_motor_speed(GO_FWD)
                elif ir_value[SECOND_IR] < 150:
                    update_motor_speed(GO_BWD)  
                else:
                    if rflag:
                        update_motor_speed(GO_RIGHT)
                    else:
                        update_motor_speed(GO_LEFT)  
                    steps_taken = 0
                    flag = 0
            else: #ir_value[SECOND_IR] == 1000
                if ir_value[FIRST_IR] > 250:
                    update_motor_speed(GO_FWD)
                elif ir_value[FIRST_IR] < 150:
                    update_motor_speed(GO_BWD)  
                else:
                    if rflag:
                        update_motor_speed(GO_RIGHT)
                    else:
                        update_motor_speed(GO_LEFT)  
                    steps_taken = 0
                    flag = 0
        return
    
    is_facing_output = is_facing(angle_robot_north, goal_angle)
    if is_facing_output == 1:
        if steps_taken == 100: #is facing the right side of the path
            if obs_infront(ir_value):
                steps_taken = 101
                seen_obs_state = 4
            else:
                goal_angle = rob_ang_prev
                steps_taken = 101
        else: #steps_taken == 101 and is facing wall again 
            if finished_turn == 1:
                if ir_value[FIRST_IR] > sensor2_before_turn or ir_value[SECOND_IR] > sensor5_before_turn:
                    update_motor_speed(GO_FWD)
                else:
                    starting_turn = 1
                    finished_turn = 0
                    flag = 1
                    sensor2_before_turn = -1
                    sensor5_before_turn = -1
                    
    else: # is_facing_output != 1
        if starting_turn == 1: #just started turning to the right
            if sensor2_before_turn == -1 and sensor5_before_turn == -1:
                sensor2_before_turn = ir_value[FIRST_IR]
                sensor5_before_turn = ir_value[SECOND_IR]
            
            if ir_value[FIRST_IR] < 950 or ir_value[SECOND_IR] < 950:
                update_motor_speed(GO_BWD) 
            else:
                starting_turn = 0
                finished_turn = 1
        else:
            if is_facing_output == -1:
                update_motor_speed(GO_CCW) #rotate counter clockwise
            else: # is_facing_output == 0
                update_motor_speed(GO_CW) #rotate clockwise       
    return

def follow_wall(gps_values, compass_val, sonar_value, rflag): 
    global seen_obs_state
    global steps_taken
    global goal_angle
    global rob_ang_prev
    global FIRST_IR
    global SECOND_IR
    
    FIRST_IR = 2 if rflag else 5
    SECOND_IR = 5 if rflag else 2
    angle_robot_north = math.atan2(compass_val[1], compass_val[0]) #atan(y/x)  
    
    # print(seen_obs_state)
    
    if seen_obs_state == -1:
        if steps_taken < 100:
            if ir_value[FIRST_IR] < 1000: # if the wall had a little openning in it
                seen_obs_state = 1  
                steps_taken = 0
                return
            if rflag:
                update_motor_speed(GO_RIGHT)
            else:
                update_motor_speed(GO_LEFT)
            steps_taken = steps_taken + 1
            if steps_taken == 100:
                goal_angle = angle_robot_north + 1.57 if rflag \
                        else angle_robot_north - 1.57
        else: #steps_taken >= 100
            is_facing_output = is_facing(angle_robot_north, goal_angle) 
            if is_facing_output == 1:              
                if rflag:
                    update_motor_speed(GO_RIGHT)
                else:
                    update_motor_speed(GO_LEFT) 
                steps_taken = 0
                seen_obs_state = 0                             
            elif is_facing_output == -1:
                update_motor_speed(GO_CCW) #rotate counter clockwise
            else:
                update_motor_speed(GO_CW) #rotate clockwise 
                        
    elif seen_obs_state == 0:               
        if steps_taken < 100:
            if rflag:
                update_motor_speed(GO_RIGHT)
            else:
                update_motor_speed(GO_LEFT)
            steps_taken = steps_taken + 1 
            if ir_value[FIRST_IR] < 1000:
                seen_obs_state = 1               
            if steps_taken == 100:
                rob_ang_prev = angle_robot_north
                goal_angle = angle_robot_north - 1.57 if rflag \
                        else angle_robot_north + 1.57
        else: #steps_taken >= 100
            check_obs_in_way(ir_value, rflag)                                 
    elif seen_obs_state == 1:
        #set distance from wall
        if ir_value[FIRST_IR] == 1000 and ir_value[SECOND_IR] == 1000:
            if rflag:
                update_motor_speed(GO_RIGHT)
            else:
                update_motor_speed(GO_LEFT)
        elif ir_value[FIRST_IR] > 650:
            update_motor_speed(GO_FWD)
            return
        elif ir_value[FIRST_IR] < 600:
            update_motor_speed(GO_BWD)
            return
        else: # the distance is ok
            if rflag:
                update_motor_speed(GO_RIGHT)
            else:
                update_motor_speed(GO_LEFT)
            seen_obs_state = 2
            
        if steps_taken < 100:
            steps_taken = steps_taken + 1 
            if steps_taken == 100:
                rob_ang_prev = angle_robot_north
                goal_angle = angle_robot_north - 1.57 if rflag \
                        else angle_robot_north + 1.57
        else: # steps_taken >= 100                    
            check_obs_in_way(ir_value, rflag)
    elif seen_obs_state == 2:           
        if steps_taken < 100:
            if rflag:
                update_motor_speed(GO_RIGHT)
            else:
                update_motor_speed(GO_LEFT)
            steps_taken = steps_taken + 1 
            if ir_value[SECOND_IR] < 1000:
                seen_obs_state = 3 
                steps_taken = 0        
            if steps_taken == 100:
                rob_ang_prev = angle_robot_north
                goal_angle = angle_robot_north - 1.57 if rflag \
                        else angle_robot_north + 1.57
        else: # steps_taken >= 100                    
            check_obs_in_way(ir_value, rflag)                           
    elif seen_obs_state == 3:    
        if rflag:
            update_motor_speed(GO_RIGHT)
        else:
            update_motor_speed(GO_LEFT)
        
        if ir_value[FIRST_IR] < 1000 and ir_value[SECOND_IR] < 1000: #completely sees a wall
            seen_obs_state = 4
            steps_taken = 0
            return
           
        if ir_value[FIRST_IR] == 1000 and ir_value[SECOND_IR] == 1000: #wall has ended
            seen_obs_state = -1
            steps_taken = 0
            goal_angle = angle_robot_north + 1.57 if rflag \
                    else angle_robot_north - 1.57
    elif seen_obs_state == 4:
        if steps_taken < 100:
            if ir_value[FIRST_IR] == 1000 and ir_value[SECOND_IR] == 1000: 
                if sonar_value[0] == 1000: #wall has ended
                    seen_obs_state = -1
                    steps_taken = 0   
                else:
                     update_motor_speed(GO_FWD)                                               
            else:
                if rflag:
                    update_motor_speed(GO_RIGHT)
                else:
                    update_motor_speed(GO_LEFT)
                steps_taken = steps_taken + 1
                if steps_taken == 100:
                    rob_ang_prev = angle_robot_north
                    goal_angle = angle_robot_north - 1.57 if rflag \
                            else angle_robot_north + 1.57
        else: #steps_taken >= 100
            check_obs_in_way(ir_value, rflag)
    else: #if seen_obs_state == 5
        if ir_value[FIRST_IR] == 1000:
            if ir_value[SECOND_IR] > 650:
                update_motor_speed(GO_FWD)
            elif ir_value[SECOND_IR] < 600:
                update_motor_speed(GO_BWD) 
            else:
                if rflag:
                    update_motor_speed(GO_CCW)   
                else:
                    update_motor_speed(GO_CW) 
        elif ir_value[SECOND_IR] == 1000:
            if ir_value[FIRST_IR] > 650:
                update_motor_speed(GO_FWD)
            elif ir_value[FIRST_IR] < 600:
                update_motor_speed(GO_BWD) 
            else:
                if rflag:
                    update_motor_speed(GO_CW)   
                else:
                    update_motor_speed(GO_CCW)    
        elif abs(ir_value[FIRST_IR] - ir_value[SECOND_IR]) > 20: 
            if ir_value[FIRST_IR] > ir_value[SECOND_IR]: #right wheel is farther than the left 
                if rflag:
                    update_motor_speed(GO_CCW)   
                else:
                    update_motor_speed(GO_CW) 
            else:
                if rflag:
                    update_motor_speed(GO_CW)   
                else:
                    update_motor_speed(GO_CCW) 
        elif ir_value[FIRST_IR] > 750 or ir_value[SECOND_IR] > 750:
            update_motor_speed(GO_FWD)
        elif ir_value[FIRST_IR] < 700 or ir_value[SECOND_IR] < 700:
            update_motor_speed(GO_BWD) 
        else:
            seen_obs_state = 4               
    return 

def init_wall_following():   
    global seen_obs_state
    global steps_taken

    seen_obs_state = 5
    steps_taken = 0
    return 
    
def obs_infront(ir_value): 
    if ir_value[2] < 950 or ir_value[5] < 950:
        return 1
    return 0
    
def no_obs(): 
    global seen_obs_state
    # (seen_obs_state == -1 and cur_state == wall_following) or cur_state == forward:
    if seen_obs_state == -1:
        return 1
    return 0


def is_in_last_pos(p, q):
    if math.dist(p, q) < 0.1:
        return 1
    return 0


if __name__ == "__main__":
    
    TIME_STEP = 32
    robot = init_robot(time_step=TIME_STEP)
    init_robot_state(in_pos=[0,0,0],in_omega=[0,0,0]) 
    prev_locations = [];
    # sonar_1 = robot.getDevice("sonar_1")
    
    # DEFINE STATES HERE!
    FORWARD = 0
    WALL_FOLLOWING = 1
    ROTATE_TO_GOAL = 2
    RETURN_TO_WALL = 3
    
    cur_state = ROTATE_TO_GOAL

    prev_angle = 0
    prev_x = 0
    prev_y = 0
    
    flag = 0
    
    while robot.step(TIME_STEP) != -1:

        gps_values,compass_val,sonar_value,encoder_value,ir_value = read_sensors_values()
        update_robot_state() 
        angle_robot_north = math.atan2(compass_val[1], compass_val[0])
        angle_goal_robot = math.atan2(goal_position[1]-gps_values[1], goal_position[0]-gps_values[0])
        
        if cur_state == FORWARD:
            if abs(gps_values[1] - goal_position[1]) < 0.2 and abs(gps_values[0] - goal_position[0]) < 0.2:
                update_motor_speed([0,0,0])
                exit()
            if flag == 1 or flag == 0:
                update_motor_speed(GO_FWD)
                if flag == 1:
                    flag = 2
            elif flag == 2:
                if prev_x == gps_values[0] and prev_y == gps_values[1]:
                    cur_state = RETURN_TO_WALL
                else:
                    update_motor_speed(GO_FWD)
                    flag = 0
            if obs_infront(ir_value): 
                cur_state = WALL_FOLLOWING
                init_wall_following()
        elif cur_state == WALL_FOLLOWING:
            follow_wall(gps_values, compass_val, sonar_value, 0)
            # loc = (round(compass_val[0], 6), round(compass_val[1], 6)) 
            if no_obs() and (flag == 0 or flag == 2):
                cur_state = ROTATE_TO_GOAL
                prev_angle = angle_robot_north
                flag = 1
        elif cur_state == ROTATE_TO_GOAL:
            if rotate_to_angle(angle_robot_north, angle_goal_robot):
                if obs_infront(ir_value):
                    cur_state = RETURN_TO_WALL
                else:
                    cur_state = FORWARD
        elif cur_state == RETURN_TO_WALL:
            if rotate_to_angle(angle_robot_north, prev_angle):
                cur_state = WALL_FOLLOWING
                
        
        prev_x = gps_values[0]
        prev_y = gps_values[1]    
        
                
    pass