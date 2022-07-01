import rospy
from geometry_msgs.msg import Quaternion, Pose, PoseArray
from assistive_cane.astar import AStar
from assistive_cane.msg import cane_command_msg
from tf.transformations import euler_from_quaternion
from assistive_cane.world_visualizer import WorldVisualizer
import math

# import time

"""
Code Implementation Overview

Day 1: TODO 1

Day 2: None

Day 3: None

Day 4: TODO 2 

Day 5: Demo!
"""

"""
    [Day 4] TODO 2: Modify constants to optimize your implementation
        Refer to the documentation for cane_callback() to understand what each constant does

"""
####### Insert Code Here #######

MIN_DISTANCE_THRESHOLD = 0.225
MIN_ROTATION_THRESHOLD = math.pi/6

FORWARD_CONST_THRESHOLD = 0.3
FORWARD_MIN_VIBRATION = 0.4

ROTATION_CONST_THRESHOLD = math.pi/4
ROTATION_MIN_VIBRATION = 0.4

last_command = 0

# 0 - forward
# 1 - rotation

################################

def cane_callback(cane_pose):
    """
    Sends vibration commands to smart cane to guide it to next target waypoint which consists of target position and target orientation)

    Controller Logic:

        Deciding Command Type:

            If the position of the cane is at a distance from the target position further than MIN_DISTANCE_THRESHOLD: 
                If the orientation of the cane differs from the direction towards the target position outside MIN_ROTATION_THRESHOLD:
                    Rotate (left or right) the cane such that it faces the target position
                Else:
                    Move forward towards the target position 
            Else:
                If the orientation of the cane differs from the target orientation by more than MIN_ROTATION_THRESHOLD:
                    Rotate (left or right) the cane towards the target orientation
                Else:
                    You have reached target waypoint! Start tracking the next target waypoint (if any left)

        Deciding Command Magnitude:
        
            Move Forward:

                If the distance from target position is greater than FORWARD_CONST_THRESHOLD:
                    forward vibration magnitude = 1
                Else:
                    forward vibration magnitude = FORWARD_MIN_VIBRATION + (1-FORWARD_MIN_VIBRATION) * distance from target/FORWARD_CONST_THRESHOLD

            Rotate:

                If the orientation of the cane differs from the target orientation by more than ROTATION_CONST_THRESHOLD:
                    rotation (left or right) vibration magnitude = 1
                Else: 
                    rotation (left or right) vibration magnitude = ROTATION_MIN_VIBRATION + (1-ROTATION_MIN_VIBRATION) * distance from target/ROTATION_CONST_THRESHOLD

    Parameters
    ----------
    cane_pose: Pose
        pose of the cane in world frame

    Returns
    -------
    None 
    """
    
    def vibration_command(current_yaw, target_yaw):
        """
        Returns vibration commands to indicate direction to turn

        Parameters
        ----------
        current_yaw: float
            cane orientation in radians
        target_yaw: float
            target orientation in radians

        Returns
        -------
        tuple
            vibration command for left and right (left_command, right_command)
        """

        #  assumes both are in -pi, pi

        # Convert angles to between 0 - 2pi
        if current_yaw < 0:
            current_yaw += 2*math.pi
        if target_yaw < 0:
            target_yaw += 2*math.pi

        # TODO Rajat document this
        if current_yaw < target_yaw:

            if target_yaw - current_yaw < math.pi: #move left
                if target_yaw - current_yaw > ROTATION_CONST_THRESHOLD:
                    return (1.0, 0.0)
                else:
                    return (ROTATION_MIN_VIBRATION + (1-ROTATION_MIN_VIBRATION)*(target_yaw - current_yaw)/ROTATION_CONST_THRESHOLD, 0.0)
            else: # move right
                if current_yaw + 2*math.pi - target_yaw > ROTATION_CONST_THRESHOLD:
                    return (0.0, 1.0)
                else:
                    return (0.0, ROTATION_MIN_VIBRATION + (1-ROTATION_MIN_VIBRATION)*(current_yaw + 2*math.pi - target_yaw)/ROTATION_CONST_THRESHOLD)
        
        # TODO Rajat document this
        else:   

            if current_yaw - target_yaw < math.pi: # move right
                if current_yaw - target_yaw > ROTATION_CONST_THRESHOLD:
                    return (0.0, 1.0)
                else:
                    return (0.0, ROTATION_MIN_VIBRATION + (1-ROTATION_MIN_VIBRATION)*(current_yaw - target_yaw)/ROTATION_CONST_THRESHOLD)

            else: # move left
                if target_yaw + 2*math.pi - current_yaw > ROTATION_CONST_THRESHOLD:
                    return (1.0, 0.0)
                else:
                    return (ROTATION_MIN_VIBRATION + (1-ROTATION_MIN_VIBRATION)*(target_yaw + 2*math.pi - current_yaw)/ROTATION_CONST_THRESHOLD, 0.0)
    
    box_topic = "/boxes_state"
    local_box_poses = rospy.wait_for_message(box_topic, PoseArray, timeout=None)
    world_visualizer.visualize_boxes(local_box_poses.poses)

    global current_index
    global path 
    global last_command
    global MIN_DISTANCE_THRESHOLD
    global MIN_ROTATION_THRESHOLD

    if last_command == 0:
        MIN_DISTANCE_THRESHOLD = 0.1
        MIN_ROTATION_THRESHOLD = math.pi/6
    else:
        MIN_DISTANCE_THRESHOLD = 0.225
        MIN_ROTATION_THRESHOLD = math.pi/8


    if current_index == len(path):
        print("Successfully executed path")
        cane_command = cane_command_msg()
        cane_command.vibration_forward = 0
        cane_command.vibration_left = 0
        cane_command.vibration_right = 0
        cane_command_publisher.publish(cane_command)
        quit()

    world_visualizer.visualize_cane(cane_pose)

    marker_id = 1
    for i in range(current_index, len(path)):
        world_visualizer.visualize_cane(path[i], False, marker_id)
        marker_id += 1

    waypoint_target = path[current_index]

    print("Current Index: ",current_index)
    print("Target: ",waypoint_target)

    (_, _, cane_yaw) = euler_from_quaternion([cane_pose.orientation.x, cane_pose.orientation.y, cane_pose.orientation.z, cane_pose.orientation.w])
    (_, _, waypoint_target_yaw) = euler_from_quaternion([waypoint_target.orientation.x, waypoint_target.orientation.y, waypoint_target.orientation.z, waypoint_target.orientation.w])

    if (waypoint_target.position.x - cane_pose.position.x)**2 + (waypoint_target.position.y - cane_pose.position.y)**2 > MIN_DISTANCE_THRESHOLD**2:

        theta = math.atan2(waypoint_target.position.y - cane_pose.position.y, waypoint_target.position.x - cane_pose.position.x)

        if abs(theta - cane_yaw) > MIN_ROTATION_THRESHOLD:
            (vibration_left, vibration_right) = vibration_command(cane_yaw, theta)
            
            cane_command = cane_command_msg()
            cane_command.vibration_forward = 0
            cane_command.vibration_left = vibration_left
            cane_command.vibration_right = vibration_right
            cane_command_publisher.publish(cane_command)
            last_command = 1

        else:

            cane_command = cane_command_msg()
            cane_command.vibration_left = 0
            cane_command.vibration_right = 0

            if (waypoint_target.position.x - cane_pose.position.x)**2 + (waypoint_target.position.y - cane_pose.position.y)**2 > FORWARD_CONST_THRESHOLD**2:
                cane_command.vibration_forward = 1.0
            else:
                cane_command.vibration_forward = FORWARD_MIN_VIBRATION + (1-FORWARD_MIN_VIBRATION)*math.sqrt((waypoint_target.position.x - cane_pose.position.x)**2 + (waypoint_target.position.y - cane_pose.position.y)**2)/FORWARD_CONST_THRESHOLD

            cane_command_publisher.publish(cane_command)

            last_command = 0


    elif abs(waypoint_target_yaw - cane_yaw) > MIN_ROTATION_THRESHOLD:

        (vibration_left, vibration_right) = vibration_command(cane_yaw, waypoint_target_yaw)

        cane_command = cane_command_msg()
        cane_command.vibration_forward = 0
        cane_command.vibration_left = vibration_left
        cane_command.vibration_right = vibration_right
        cane_command_publisher.publish(cane_command)

        last_command = 1


    else:
        current_index += 1

    if current_index == len(path):
        print("Successfully executed path")
        cane_command = cane_command_msg()
        cane_command.vibration_forward = 0
        cane_command.vibration_left = 0
        cane_command.vibration_right = 0
        cane_command_publisher.publish(cane_command)
        quit()

    

if __name__ == '__main__':

    rospy.init_node('smart_cane_planner')
    # Define your image topic
    box_topic = "/boxes_state"
    cane_topic = "/cane_state"

    print('Waiting for msg')

    local_box_poses = rospy.wait_for_message(box_topic, PoseArray, timeout=None)
    start_cane_pose = rospy.wait_for_message(cane_topic, Pose, timeout=None)

    print("Box poses: ")
    print(local_box_poses)

    planner = AStar(3.4,2.5,34,25)

    # find nearest angle

    (_, _, start_pose_cane_yaw) = euler_from_quaternion([start_cane_pose.orientation.x, start_cane_pose.orientation.y, start_cane_pose.orientation.z, start_cane_pose.orientation.w])

    print("Cane Yaw: ", start_pose_cane_yaw)
    if start_pose_cane_yaw < 0:
        start_pose_cane_yaw += 2*math.pi

    if start_pose_cane_yaw < 0:
        print("euler_from_quaternion returns <0 angle: ",start_pose_cane_yaw)
        exit()

    if start_pose_cane_yaw >= math.pi/4 and start_pose_cane_yaw < 3*math.pi/4:
        start_direction = 0
    elif start_pose_cane_yaw >= 3*math.pi/4 and start_pose_cane_yaw < 5*math.pi/4:
        start_direction = 3
    elif start_pose_cane_yaw >= 5*math.pi/4 and start_pose_cane_yaw < 7*math.pi/4:
        start_direction = 2
    else:
        start_direction = 1

    print("Start Direction: ", start_direction)

    goal_cane_pose_x = 3.0
    goal_cane_pose_y = 2.0
    goal_cane_pose_direction = 1

    global path
    
    # start = time.perf_counter()
    path = planner.solve((start_cane_pose.position.x,start_cane_pose.position.y,start_direction),(goal_cane_pose_x,goal_cane_pose_y,goal_cane_pose_direction),local_box_poses.poses)
    # end = time.perf_counter()
    # print(f"total time: {start - end:'0.4f'}")

    global current_index
    current_index = 0

    global cane_command_publisher

    """
    [Day 1] TODO 1: Create cane command publisher
        - Create a publisher with the following parameters
            - Topic: "/cane_command"
            - Message type: cane_command_msg 
            - Queue size: 10

    """
    ####### Insert Code Here #######

    cane_command_publisher = rospy.Publisher('/cane_command', cane_command_msg, queue_size=10)

    ################################

    global world_visualizer
    world_visualizer = WorldVisualizer()
    world_visualizer.visualize_boxes(local_box_poses.poses)

    rospy.Subscriber(cane_topic, Pose, cane_callback)
    rospy.spin()
