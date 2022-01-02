import sys
import numpy as np
from copy import deepcopy

import rospy
import roslib

from collections import namedtuple
from math import pi
import uuid
import math
import time
import random

# Common interfaces for interacting with both the simulation and real environments!
from core.interfaces import ArmController
from core.interfaces import ObjectDetector

# for timing that is consistent with simulation or real time as appropriate
from core.utils import time_in_seconds

# The library you implemented over the course of this semester!
from lib.calculateFK import FK
from lib.calcJacobian import calcJacobian
from lib.solveIK import IK
from lib.rrt import rrt
from lib.astar import Astar
from lib.loadmap import loadmap

static_tags = ['tag1', 'tag2', 'tag3', 'tag4', 'tag5', 'tag6']
dynamic_tags = ['tag7', 'tag8', 'tag9', 'tag10', 'tag11', 'tag12']

quadrant = ['first', 'second', 'third', 'fourth']

lower_limit = np.array([-2.8973,-1.7628,-2.8973,-3.0718,-2.8973,-0.0175,-2.8973])
upper_limit = np.array([2.8973,1.7628,2.8973,-0.0698,2.8973,3.7525,2.8973])

grab_force = 50

dyn_angle_upper_limit = 0.18
dyn_angle_lower_limit = 0 

def get_camera_in_robot_base(tag0_in_camera):
    tag0_in_robot_base = np.array([[1,0,0,-0.5],[0,1,0,0],[0,0,1,0],[0,0,0,1]])

    R_tag0_in_camera = tag0_in_camera[0:3, 0:3]
    R_tag0_in_camera_inv = np.linalg.inv(R_tag0_in_camera)

    d_tag0_in_camera = tag0_in_camera[0:3, 3]
    d_tag0_in_camera_inv = np.dot((-R_tag0_in_camera_inv), d_tag0_in_camera)

    camera_in_tag0 = np.zeros((4, 4))
    camera_in_tag0[0:3, 0:3] = R_tag0_in_camera_inv
    camera_in_tag0[0:3, 3] = d_tag0_in_camera_inv
    camera_in_tag0[3, 3] = 1
    camera_in_robot_base = np.dot(tag0_in_robot_base, camera_in_tag0)
    return camera_in_robot_base


def generate_map(tags_dict_in_robot_base, team):
    # Format of obstacle: [xmin,ymin,zmin,xmax,ymax,zmax]
    obstacles = []

    # Add obstacle to prevent reaching below 0.2m in the world frame and roughly taking account for obstacles
    obstacles = np.array([[-15,-15,0,-0.1,-0.1,0.24]])
    obstacles = np.append(obstacles, np.array([[15,15,0,0.1,0.1,0.2]]), axis=0)
    obstacles = np.append(obstacles, np.array([[-15,15,0,-0.1,0.1,0.2]]), axis=0)
    obstacles = np.append(obstacles, np.array([[15,-15,0,0.1,-0.1,0.2]]), axis=0)

    MyStruct = namedtuple("map", "obstacles")
    return MyStruct(obstacles = obstacles)


def tag_detection():
    detected_tags = {}
    for (name, pose) in detector.get_detections():
        print(name,'\n',pose)
        if name != "tag0":
            detected_tags[name, str(uuid.uuid4())] = pose
        else:
            detected_tags[name] = pose
    return detected_tags


def find_dyn_and_robot_base_to_world(tags_dict_in_robot_base, robot_base_in_world):
    dyn_tag_dict_in_world = {}
    for key, value in tags_dict_in_robot_base.items():
            # Find dynamic blocks
            if key[0] in dynamic_tags:
                value[0:3, 0:3] = np.array([[1,0,0],[0,1,0],[0,0,1]])
                dyn_tag_dict_in_world[key] = np.dot(robot_base_in_world, value)
    return dyn_tag_dict_in_world


def transform_tag_from_cam_frame_to_robot_frame(tags_dict_in_camera, camera_in_robot_base):
    tags_dict_in_robot_base = {}
    for key, value in tags_dict_in_camera.items():
        tags_dict_in_robot_base[key] = np.dot(camera_in_robot_base, value)
        tags_dict_in_robot_base[key][2, 3] = 0.2 + 0.06 # manual set tag position in z
        print("transformed h matrix: ", tags_dict_in_robot_base)
    return tags_dict_in_robot_base

def determine_quadrant_index(x, y):
    if x > 0 and y > 0:
        return 0
    elif x < 0 and y > 0:
        return 1
    elif x < 0 and y < 0:
        return 2
    elif x > 0 and y < 0:
        return 3



if __name__ == "__main__":
    try:
        team = rospy.get_param("team") # 'red' or 'blue'
    except KeyError:
        print('Team must be red or blue - make sure you are running final.launch!')
        exit()

    rospy.init_node("team_script")
    arm = ArmController()
    detector = ObjectDetector()

    arm.safe_move_to_position(arm.neutral_position()) # on your mark!

    print("\n****************")
    if team == 'blue':
        print("** BLUE TEAM  **")
    else:
        print("**  RED TEAM  **")
    print("****************")
    input("\nWaiting for start... Press ENTER to begin!\n") # get set!
    print("Go!\n") # go!

    # STUDENT CODE HERE






    # CALCULATION
    

    # Define goal area
    goal_area = np.zeros((4, 4))
    goal_area[0:3, 0:3] = np.array([[1,0,0],[0,1,0],[0,0,-1]])
    goal_area[3, 3] = 1

    if team == 'blue':
        goal_area[0:3, 3] = np.array([0.56, -0.16, 0.230])
    else:
        goal_area[0:3, 3] = np.array([0.56, 0.16, 0.230])

    
    
    # Detect some tags(in camera frame)
    tags_dict_in_camera = tag_detection()
    
    # Tags representation in robot base frame
    camera_in_robot_base = get_camera_in_robot_base(tags_dict_in_camera['tag0'])
    tags_dict_in_robot_base = transform_tag_from_cam_frame_to_robot_frame(tags_dict_in_camera, camera_in_robot_base)

    # Generate map (put blocks as obstacles)
    map_struct = generate_map(tags_dict_in_robot_base, team)  

    # Find corresponding configurations for each blocks
    ik = IK()
    seed = np.array([0,0,0,-pi/2,0,pi/2,pi/4])


    
    blocks_configs = []
    tags_indices = []
    rot_offsets = []
    for key, value in tags_dict_in_robot_base.items():
        print('tag on the block: \n', key)
        print("blocks in robot base frame: \n", value)

        # Find static blocks
        if key[0] in static_tags:
            # Calculate rotational offset (around z axis) for each static block
            qw = (math.sqrt(1 + value[0,0] + value[1,1] + value[2,2])) / 2
            qz = (value[1,0] - value[0,1])/(4*qw)

            rot_offset = math.acos(qw) * 2
            if qz > 0:
                rot_offsets.append(rot_offset)
            else:
                rot_offsets.append(-rot_offset)
            print(rot_offset)

            value[0:3, 0:3] = np.array([[1,0,0],[0,1,0],[0,0,-1]])
            print("block matrix: \n")
            print(value)
            q, success, rollout = ik.inverse(value, seed)
      
            blocks_configs.append(q)
            tags_indices.append(key[0])
    



    """
    Turnable
    """
    dyn_start = arm.neutral_position()

    if team == 'blue':
        dyn_start = dyn_start + [-pi/2,0,0,0,0,0,0]
    else:
        dyn_start = dyn_start + [pi/2,0,0,0,0,0,0]
        
    turnable = np.zeros((4,4))
    turnable[3, 3] = 1
    turnable[0:3, 0:3] = np.array([[1,0,0],[0,1,0],[0,0,-1]])
    if team == 'blue':
        turnable[0:3, 3] = np.array([0, -0.73, 0.230])
    else:
        turnable[0:3, 3] = np.array([0, 0.73, 0.230])
    
    t_q, t_success, t_rollout = ik.inverse(turnable, dyn_start)






            
     










    # Generate static_goal locations
    # Generate static goal positions
    goal_locations = []
    goal_locations.append(goal_area + np.array([[0, 0, 0, 0.06],\
                                                 [0, 0, 0, 0.06],\
                                                 [0, 0, 0, 0],\
                                                 [0, 0, 0, 0]]))

    goal_locations.append(goal_area + np.array([[0, 0, 0, -0.06],\
                                                 [0, 0, 0, -0.06],\
                                                 [0, 0, 0, 0],\
                                                 [0, 0, 0, 0]]))

    goal_locations.append(goal_area + np.array([[0, 0, 0, -0.06],\
                                                 [0, 0, 0, 0.06],\
                                                 [0, 0, 0, 0],\
                                                 [0, 0, 0, 0]]))

    goal_locations.append(goal_area + np.array([[0, 0, 0, 0.06],\
                                                 [0, 0, 0, -0.06],\
                                                 [0, 0, 0, 0],\
                                                 [0, 0, 0, 0]]))

    # Generate static goal rotations
    # Unfinished
    """
    for n in range(len(tags_indices)):
        if tags_indices[n] == 'tag1':
            pass
        elif tags_indices[n] == 'tag2':
            goal_locations[n][0:3, 0:3] = np.array([[1,0,0],[0,0.707,-0.707],[0,0.707,0.707]])
        elif tags_indices[n] == 'tag3':
            goal_locations[n][0:3, 0:3] = np.array([[1,0,0],[0,1,0],[0,0,-1]])
    """


    # Generate static goal configs
    goals_configs = []
    for loc in goal_locations:
        print("loc: \n")
        print(loc)
        goal_config, g_success, g_rollout = ik.inverse(loc, seed)
        goals_configs.append(goal_config)
    
    
    
    
    

    


    filtered_predicted_dyn_blocks_in_robot_base = {}
    shortest_id = None




    # OPERATION
    arm.exec_gripper_cmd(0.09, grab_force)
    # Path planning and moving
    for n in range(len(blocks_configs)):
        print(tags_indices[n])
        # Grab block
        arm.safe_move_to_position(arm.neutral_position())

        start_grabbing = time.time()

        #path = rrt(deepcopy(map_struct), deepcopy(arm.neutral_position()), deepcopy(q))
        #for config in path:
        arm.safe_move_to_position(blocks_configs[n])
        #arm.safe_move_to_position(path[-1] + [0,0.065,0,0,0,0,0])
        arm.safe_move_to_position(blocks_configs[n] + [0,0.065,0,0,0,0, -rot_offsets[n]])

        end_grabbing = time.time()

        arm.exec_gripper_cmd(0.049, grab_force)

        grabbing_time = end_grabbing - start_grabbing

        print("\n\n\n\n\ntime grabbing: ", grabbing_time)
        

        # Back to neutral
        arm.safe_move_to_position(arm.neutral_position())

        # Drop at goal area
        arm.safe_move_to_position(goals_configs[n])
        arm.exec_gripper_cmd(0.09, grab_force)
        arm.safe_move_to_position(goals_configs[n] + [0,-0.065,0,0,0,0,0])
        #goal_area[3, 3] = goal_area[3, 3] + 0.05   # Might be useful when developing dynamic blocks

        # First detection for dynamic blocks
        tags_dict_in_camera_first_obs = tag_detection()
        first_observation = time.time()


        # Back to neutral
        arm.safe_move_to_position(arm.neutral_position())



        # Dynamic Grabber
        # Operation on dynamic blocks
        # Observe tags again and calculate angular motion for dynamic tags
        tags_dict_in_camera_second_obs = tag_detection()
        second_observation = time.time()

        time_elapsed = second_observation - first_observation

        print("\n\n\n\n\ntime elapsed: ", time_elapsed)
        #print("\n\n\n\n\nnew obseravtions: ", tags_dict_in_camera_second_obs)

        print("\n\n\n\n\nfirst detection: ",tags_dict_in_camera_first_obs)
        print("\n\n\n\n\nsecond detection: ",tags_dict_in_camera_second_obs)

        # To robot base frame
        tags_dict_robot_first_obs = transform_tag_from_cam_frame_to_robot_frame(tags_dict_in_camera_first_obs, camera_in_robot_base)
        tags_dict_robot_second_obs = transform_tag_from_cam_frame_to_robot_frame(tags_dict_in_camera_second_obs, camera_in_robot_base)


        # Robot arm frame in world frame
        robot_base_in_world = np.zeros((4,4))
        world_in_robot_base = np.zeros((4,4))
        if team == 'blue':
            robot_base_in_world = np.array([[1, 0, 0, 0],\
                                            [0, 1, 0, 0.978],\
                                            [0, 0, 1, 0],\
                                            [0, 0, 0, 1]])

            world_in_robot_base = np.array([[0, 1, 0, 0],\
                                            [1, 0, 0, -0.978],\
                                            [0, 0, -1, 0],\
                                            [0, 0, 0, 1]])        
        else:
            robot_base_in_world = np.array([[1, 0, 0, 0],\
                                            [0, 1, 0, -0.978],\
                                            [0, 0, 1, 0],\
                                            [0, 0, 0, 1]])

            world_in_robot_base = np.array([[0, 1, 0, 0],\
                                            [1, 0, 0, 0.978],\
                                            [0, 0, -1, 0],\
                                            [0, 0, 0, 1]])

        # Arm frame to world frame
        tag_dict_world_first_obs = find_dyn_and_robot_base_to_world(tags_dict_robot_first_obs, robot_base_in_world)
        tag_dict_world_second_obs = find_dyn_and_robot_base_to_world(tags_dict_robot_second_obs, robot_base_in_world)
        
        print("\n\n\n\n\ntag_dict_world_first_obs: ", tag_dict_world_first_obs)
        print("\n\n\n\n\ttag_dict_world_second_obs: ", tag_dict_world_second_obs)

        # Pair up
        
        shortest_dist = 10

        for first_key, first_value in tag_dict_world_first_obs.items():
            first_x = first_value[0, 3]
            first_y = first_value[1, 3]
            first_angle = math.atan(first_y/first_x)
            first_quadrant_index = determine_quadrant_index(first_x, first_y)
            for second_key, second_value in tag_dict_world_second_obs.items():
                if first_key[0] == second_key[0]:
                    second_x = second_value[0, 3]
                    second_y = second_value[1, 3]
                    second_angle = math.atan(second_y/second_x)        
                    second_quadrant_index = determine_quadrant_index(second_x, second_y)

                    if (first_quadrant_index == second_quadrant_index) or (quadrant[second_quadrant_index-1] == quadrant[first_quadrant_index]):
                        angle_difference = second_angle - first_angle


                        
                        if(angle_difference > dyn_angle_lower_limit) and (angle_difference < dyn_angle_upper_limit):
                            print("\n\n",second_key, "\n\n")
                            print("\n\nangle diff: ", angle_difference)
                            print(quadrant[first_quadrant_index], quadrant[second_quadrant_index])

                            angular_velocity = angle_difference / time_elapsed
                            angle_addition = angular_velocity * grabbing_time
                            predicted_angle = second_angle + angle_addition

                            r = math.sqrt((second_y**2)+(second_x**2))

                            new_x = r * math.cos(predicted_angle)
                            new_y = r * math.sin(predicted_angle)

                            new_pos_in_world = np.array([[1, 0, 0, new_x],\
                                                         [0, 0, 0, new_y],\
                                                         [0, 0, 1, 0.3],\
                                                         [0, 0, 0, 1]])
                            
                            new_pos_in_robot_base = np.dot(world_in_robot_base, new_pos_in_world)
                            filtered_predicted_dyn_blocks_in_robot_base[second_key[1]] = new_pos_in_robot_base

                            if shortest_dist > (np.linalg.norm(new_pos_in_robot_base[0:3, 3])):
                                shortest_dist = np.linalg.norm(new_pos_in_robot_base[0:3, 3])
                                shortest_id = second_key[1]

        dyn_block_q, db_success, db_rollout = ik.inverse(filtered_predicted_dyn_blocks_in_robot_base[shortest_id], seed)

        print("\n\nThe shortest id is: ", shortest_id)
        print("\n\nThe distance is: ", shortest_dist)
        print("\n\nThe H Matrix is: ", filtered_predicted_dyn_blocks_in_robot_base[shortest_id])


        
        arm.safe_move_to_position(dyn_block_q)
        """            

        
        # Pseudo Dynamic Grabber
        """
        arm.safe_move_to_position(dyn_start)      
        arm.safe_move_to_position(t_q + [random.uniform(-0.3, 0.3),-0.055,0,0,0,0,0])
        arm.exec_gripper_cmd(0.049, grab_force)

        arm.safe_move_to_position(goals_configs[n] + [0,-0.095,0,0,0,0,0])
        arm.exec_gripper_cmd(0.09, grab_force)

        arm.safe_move_to_position(arm.neutral_position())



        
       
    # END STUDENT CODE