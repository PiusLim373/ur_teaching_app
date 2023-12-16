#!/usr/bin/env python

# action server here
# from builtins import print, range
from yaml import dump
import roslib
roslib.load_manifest('ur10_pap')
import rospy
from ur10_pap.msg import *
from ur10_gripper.msg import *
import actionlib
import requests
from flask import json
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi, sin
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from time import sleep
import rviz_tools_py as rviz_tools
import threading
import pose_generator
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped, WrenchStamped
from ur_msgs.msg import IOStates
import os
import random
import time
import tf
import socket

speed_scale = 1
markers = rviz_tools.RvizMarkers('/world', 'visualization_marker')
target_markers = rviz_tools.RvizMarkers('/world', 'target_marker')
coordinate_wrt_world = geometry_msgs.msg.PoseStamped()
hasreturn = False
place_coor = [0, 0.8, 1.1, 0.707, 0.707, 0, 0]
home_coor = [0.36222, 0.1744, 0.8482, -1, 0, 0, 0]

PARALLEL = 50
SUCTION = 51
DRAG = 52

picking_joint = None
current_gripper = "parallel_gripper_manipulator"
table_height = 0.9

disable_gripper = False
gripper_client = None


UR_DASHBOARD_SERVER_IP = '192.168.10.101'
UR_DASHBOARD_SERVER_PORT = 29999
UR_SOCKET_SERVER_PORT = 30002

class MoveGroupPythonInteface(object):

    item_added_to_scene = []
    def __init__(self):
        global transform_server_pub
        super(MoveGroupPythonInteface, self).__init__()

        moveit_commander.roscpp_initialize(sys.argv)
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()

        parallel_gripper_manipulator = moveit_commander.MoveGroupCommander("parallel_gripper_manipulator")
        suction_gripper_manipulator = moveit_commander.MoveGroupCommander("suction_gripper_manipulator")
        drag_gripper_manipulator = moveit_commander.MoveGroupCommander("drag_gripper_manipulator")

        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory,queue_size=20)

        self.box_name = ''
        self.robot = robot
        self.scene = scene

        self.parallel_gripper_manipulator = parallel_gripper_manipulator
        self.suction_gripper_manipulator = suction_gripper_manipulator
        self.drag_gripper_manipulator = drag_gripper_manipulator

################################################################################## Gripper independant ###########
    # def display_marker(self, marker_position, marker_orien, step):
    #     global markers
    #     while True:
    #         global stop_marker
    #         i = 0
    #         k = 0
    #         for i in range(step):
    #             pose = Pose(Point(marker_position[k], marker_position[k+1], marker_position[k+2]), Quaternion(marker_orien[0], marker_orien[1], marker_orien[2], marker_orien[3]))
    #             axis_length = 0.1
    #             axis_radius = 0.01
    #             markers.publishAxis(pose, axis_length, axis_radius, 5.0)
    #             k += 3
    #         if stop_marker:
    #             break
    #     return 1

    def display_marker(self, marker_position, marker_orien, step):
        global markers
        i = 0
        k = 0
        for i in range(step):
            pose = Pose(Point(marker_position[k], marker_position[k+1], marker_position[k+2]), Quaternion(marker_orien[0], marker_orien[1], marker_orien[2], marker_orien[3]))
            axis_length = 0.1
            axis_radius = 0.01
            markers.publishAxis(pose, axis_length, axis_radius, 5.0)
            k += 3
        return 1

    def cleanup_node(self):
        global markers
        markers.deleteAllMarkers()
        return 1

    def attach_item(self, item, gripper):
        box_name = item
        robot = self.robot
        scene = self.scene
        eef_link = eval("self." + gripper).get_end_effector_link()
        touch_links = eval("self." + gripper).get_end_effector_link()
        print(eef_link)
        print("attaching item")
        scene.attach_box(eef_link, box_name, touch_links=touch_links)
        time.sleep(1)
        return True

    def detach_item(self, item, gripper):
        box_name = item
        scene = self.scene
        eef_link = eval("self." + gripper).get_end_effector_link()
        print(eef_link)
        print("detaching item")
        scene.remove_attached_object(eef_link, name=box_name)
        time.sleep(1)
        return True

    def add_item(self, item, position, size):
        scene = self.scene
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "parallel_gripper_grasp"
        box_pose.pose.position.x = position[0]
        box_pose.pose.position.y = position[1]
        box_pose.pose.position.z = position[2]
        box_name = item
        self.item_added_to_scene.append(item)
        scene.add_box(box_name, box_pose, size=(size[0], size[1], size[2]))
        self.box_name=box_name
        return True     

    def add_stl(self, item, position, filename):
        scene = self.scene
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "world"
        box_pose.pose.position.x = position[0]
        box_pose.pose.position.y = position[1]
        box_pose.pose.position.z = position[2]
        box_pose.pose.orientation.x = position[3]
        box_pose.pose.orientation.y = position[4]
        box_pose.pose.orientation.z = position[5]
        box_pose.pose.orientation.w = position[6]
        box_name = item
        scene.add_mesh(box_name, box_pose, filename, size=(0.001, 0.001, 0.001))
        self.box_name=box_name
        return True  

    def remove_item(self, item):
        scene = self.scene
        scene.remove_world_object(item)
        return True

################################################################################## Gripper dependant ###########
    
    def gripper_control(self, action):
        if disable_gripper: 
            return True
        gripper_goal = GripperServerGoal()
        if action == "grip":
            gripper_goal.task = gripper_goal.GRIPPER_ACTION
            gripper_goal.option = gripper_goal.CLOSE_GRIPPER
        elif action == "release":
            gripper_goal.task = gripper_goal.GRIPPER_ACTION
            gripper_goal.option = gripper_goal.OPEN_GRIPPER
        try:
            gripper_client.send_goal(gripper_goal)
            if gripper_client.wait_for_result(timeout=rospy.Duration(5.0)):
                gripper_result = GripperServerResult()
                gripper_result = gripper_client.get_result()
                if gripper_result.success:
                    return gripper_result.opening
                else:
                    return False
            else:
                rospy.logwarn("Gripper server timeout, signal to re-open gripper server")
                # Code to reopen gripper server
                #
                #
                #
                return False
        except:
            return False

    def homing(self):
        group = eval("self." + current_gripper)
        joint_goal = group.set_named_target("wetbay_home")
        go_result = group.go(joint_goal, wait=True)
        group.stop()
        return go_result

    def drybay_homing(self):
        group = eval("self." + current_gripper)
        joint_goal = group.set_named_target("drybay_home")
        go_result = group.go(joint_goal, wait=True)
        group.stop()
        return go_result
    
    def pt1_homing(self):
        group = eval("self." + current_gripper)
        joint_goal = group.set_named_target("pt1_home")
        go_result = group.go(joint_goal, wait=True)
        group.stop()
        return go_result

    def pt1_instrument_pap_homing(self):
        group = eval("self." + current_gripper)
        joint_goal = group.set_named_target("pt1_instrument_pap")
        go_result = group.go(joint_goal, wait=True)
        group.stop()
        return go_result

    def pt1_fail_pick_release_homing(self):
        group = eval("self." + current_gripper)
        joint_goal = group.set_named_target("pt1_fail_pick_release")
        go_result = group.go(joint_goal, wait=True)
        group.stop()
        return go_result

    def pt2_homing(self):
        group = eval("self." + current_gripper)
        joint_goal = group.set_named_target("pt2_home")
        go_result = group.go(joint_goal, wait=True)
        group.stop()
        return go_result

    def wetbay_homing(self):
        group = eval("self." + current_gripper)
        joint_goal = group.set_named_target("wetbay_home")
        go_result = group.go(joint_goal, wait=True)
        group.stop()
        return go_result

    def move_to_saved_joint(self, saved_name):
        group = eval("self." + current_gripper)
        joint_goal = group.set_named_target(saved_name)
        go_result = group.go(joint_goal, wait=True)
        group.stop()
        return go_result

    def move_with_joint(self, joint, angle, manipulator = "parallel_gripper_manipulator"):
        group = eval("self." + manipulator)
        joint_goal = group.get_current_joint_values()
        joint_goal[joint] = angle
        go_result = group.go(joint_goal, wait=True)
        group.stop()
        return go_result
    
    def move_all_joint(self, angle, manipulator = "parallel_gripper_manipulator"):
        group = eval("self." + manipulator)
        joint_goal = group.get_current_joint_values()
        joint_goal[0] = angle[0]
        joint_goal[1] = angle[1]
        joint_goal[2] = angle[2]
        joint_goal[3] = angle[3]
        joint_goal[4] = angle[4]
        joint_goal[5] = angle[5]
        go_result = group.go(joint_goal, wait=True)
        group.stop()
        return go_result

    def plan_cartesian_path(self, axis, direction, distance, collisions_bool = True, gripper = "parallel_gripper_manipulator"):
        # return False
        group = eval("self." + gripper)
        waypoints = []
        wpose = group.get_current_pose().pose
        if(axis == "x"):
            if(direction == "up"):
                wpose.position.x += distance
            else: 
                wpose.position.x -= distance
        elif(axis == "y"):
            if(direction == "up"):
                wpose.position.y += distance
            else: 
                wpose.position.y -= distance
        elif(axis == "z"):
            if(direction == "up"):
                wpose.position.z += distance
            else: 
                wpose.position.z -= distance
        waypoints.append(copy.deepcopy(wpose))
        (path, fraction) = group.compute_cartesian_path(waypoints, 0.01, 0, avoid_collisions=collisions_bool)
        
        if fraction < 0.90:
            return False

        else:
            print("fraction = " + str(fraction))
            # check if shoulder_base rotates more then 100degree or 1.745rad
            # print(path)
            num_point = len(path.joint_trajectory.points)
            for i in range(num_point):
                if i > 0:
                    check_joint_0 = abs(path.joint_trajectory.points[i].positions[0] - path.joint_trajectory.points[0].positions[0])
                    check_joint_2 = path.joint_trajectory.points[i].positions[2]
                    check_joint_3 = abs(path.joint_trajectory.points[i].positions[3] - path.joint_trajectory.points[0].positions[3])
                    if ((check_joint_0 > 1.745) or (check_joint_2 <= 0) or (check_joint_3 > 0.3)):
                        rospy.logwarn("Large turning is predicted, aborting")
                        return False
            # for i in range(num_point):
            #     print("wrist1: ", path.joint_trajectory.points[i].positions[3])
            # for i in range(num_point):
            #     print("wrist2: ", path.joint_trajectory.points[i].positions[4])
            # for i in range(num_point):
            #     print("wrist3: ", path.joint_trajectory.points[i].positions[5])
            speedup_path = self.speedup(path)
            execute_result = group.execute(speedup_path, wait=True)
        return execute_result

    def plan_cartesian_path_new(self, axis, direction, distance, collisions_bool = True, gripper = "parallel_gripper_manipulator", ignore_large_angle_checking = False):
        replan = 0
        for replan in range(3):
            group = eval("self." + gripper)
            waypoints = []
            wpose = group.get_current_pose().pose
            if(axis == "x"):
                if(direction == "up"):
                    wpose.position.x += distance
                else: 
                    wpose.position.x -= distance
            elif(axis == "y"):
                if(direction == "up"):
                    wpose.position.y += distance
                else: 
                    wpose.position.y -= distance
            elif(axis == "z"):
                if(direction == "up"):
                    wpose.position.z += distance
                else: 
                    wpose.position.z -= distance
            waypoints.append(copy.deepcopy(wpose))
            (path, fraction) = group.compute_cartesian_path(waypoints, 0.01, 0, avoid_collisions=collisions_bool)

            if ignore_large_angle_checking and fraction >= 0.95:
                rospy.logwarn("ignoring large angle check engaged!")
                speedup_path = self.speedup(path)
                group.execute(speedup_path, wait=True)
                return True

            if fraction >= 0.9:
                print("fraction = " + str(fraction))
                # check if shoulder_base rotates more then 100degree or 1.745rad
                # print(path)
                num_point = len(path.joint_trajectory.points)
                large_turning = False
                for i in range(num_point):
                    if i > 0:
                        check_joint_0 = abs(path.joint_trajectory.points[i].positions[0] - path.joint_trajectory.points[0].positions[0])
                        check_joint_2 = path.joint_trajectory.points[i].positions[2]
                        check_joint_3 = abs(path.joint_trajectory.points[i].positions[3] - path.joint_trajectory.points[0].positions[3])
                        if ((check_joint_0 > 1.745) or (check_joint_2 <= 0) or (check_joint_3 > 0.3)):
                            large_turning = True
                            rospy.logwarn("Large turning is predicted, retrying")
                            break
                if not large_turning:
                    speedup_path = self.speedup(path)
                    execute_result = group.execute(speedup_path, wait=True)
                    return execute_result
        return False

    def speedup(self, path):
        speedup_path = moveit_msgs.msg.RobotTrajectory()
        speedup_path.joint_trajectory = path.joint_trajectory
        n_joints = len(path.joint_trajectory.joint_names)
        n_points = len(path.joint_trajectory.points)

        for i in range(n_points):
            path.joint_trajectory.points[i].time_from_start = path.joint_trajectory.points[i].time_from_start / speed_scale
            v = list(speedup_path.joint_trajectory.points[i].velocities)
            a = list(speedup_path.joint_trajectory.points[i].accelerations)
            p = list(speedup_path.joint_trajectory.points[i].positions)
            # print(v)
            for j in range(n_joints):
                v[j] = path.joint_trajectory.points[i].velocities[j] * speed_scale
                a[j] = path.joint_trajectory.points[i].accelerations[j] * speed_scale
                p[j] = path.joint_trajectory.points[i].positions[j]
                
            speedup_path.joint_trajectory.points[i].velocities = tuple(v)
            speedup_path.joint_trajectory.points[i].accelerations = tuple(a)
            speedup_path.joint_trajectory.points[i].positions = tuple(p)
        return speedup_path

    def plan_and_move(self, end_pose, collision_bool = True, gripper = "parallel_gripper_manipulator", skip_large_turn_check = False):
        global stop_marker
        group = eval("self." + gripper)
        joint = group.get_current_pose()
        replan_counter = 0

        start_position = [joint.pose.position.x, joint.pose.position.y, joint.pose.position.z]
        start_orien = [joint.pose.orientation.x, joint.pose.orientation.y, joint.pose.orientation.z, joint.pose.orientation.w]
        end_position = [end_pose[0], end_pose[1], end_pose[2]]
        end_orien = [end_pose[3], end_pose[4], end_pose[5], end_pose[6]]

        marker_position, step = pose_generator.main(start_position, end_position)
        stop_marker = False
        t = threading.Thread(target = self.display_marker, args =(marker_position, end_orien, step,))
        t.start()
        k = 0
        i = 0
        pose_list = [geometry_msgs.msg.Pose() for x in xrange(step)]
        while i in range(step):
            pose_list[i].orientation.x = end_orien[0]
            pose_list[i].orientation.y = end_orien[1]
            pose_list[i].orientation.z = end_orien[2]
            pose_list[i].orientation.w = end_orien[3]
            pose_list[i].position.x = marker_position[k]
            pose_list[i].position.y = marker_position[k+1]
            pose_list[i].position.z = marker_position[k+2]
            k += 3
            i += 1

        path, fraction = group.compute_cartesian_path(pose_list, 0.01, 0.0, avoid_collisions=collision_bool)


        if fraction < 0.90:
            return False

        else:
            if not skip_large_turn_check:
                # check if shoulder_base rotates more then 100degree or 1.745rad
                # print(path)
                num_point = len(path.joint_trajectory.points)
                for i in range(num_point):
                    if i > 0:
                        check_joint_0 = abs(path.joint_trajectory.points[i].positions[0] - path.joint_trajectory.points[0].positions[0])
                        check_joint_2 = path.joint_trajectory.points[i].positions[2]
                        if ((check_joint_0 > 1.745) or (check_joint_2 <= 0)):
                            rospy.logwarn("Large turning is predicted, aborting")
                            return False
                # for i in range(num_point):
                #     print("wrist1: ", path.joint_trajectory.points[i].positions[3])
                # for i in range(num_point):
                #     print("wrist2: ", path.joint_trajectory.points[i].positions[4])
                # for i in range(num_point):
                #     print("wrist3: ", path.joint_trajectory.points[i].positions[5])
            speedup_path = self.speedup(path)
            group.execute(speedup_path, wait=True)

        stop_marker = True
        t.join()
        self.cleanup_node()

        return True
    
    def plan_and_move_new(self, end_pose, collisions_bool = True, gripper = "parallel_gripper_manipulator", ignore_large_angle_checking = False):
        for replan in range(3):
            global stop_marker
            group = eval("self." + gripper)
            joint = group.get_current_pose()
        
            start_position = [joint.pose.position.x, joint.pose.position.y, joint.pose.position.z]
            start_orien = [joint.pose.orientation.x, joint.pose.orientation.y, joint.pose.orientation.z, joint.pose.orientation.w]
            end_position = [end_pose[0], end_pose[1], end_pose[2]]
            end_orien = [end_pose[3], end_pose[4], end_pose[5], end_pose[6]]

            marker_position, step = pose_generator.main(start_position, end_position)
            stop_marker = False
            t = threading.Thread(target = self.display_marker, args =(marker_position, end_orien, step,))
            t.start()
            k = 0
            i = 0
            pose_list = [geometry_msgs.msg.Pose() for x in xrange(step)]
            while i in range(step):
                pose_list[i].orientation.x = end_orien[0]
                pose_list[i].orientation.y = end_orien[1]
                pose_list[i].orientation.z = end_orien[2]
                pose_list[i].orientation.w = end_orien[3]
                pose_list[i].position.x = marker_position[k]
                pose_list[i].position.y = marker_position[k+1]
                pose_list[i].position.z = marker_position[k+2]
                k += 3
                i += 1

            path, fraction = group.compute_cartesian_path(pose_list, 0.01, 0.0, avoid_collisions=collisions_bool)
            
            if ignore_large_angle_checking and fraction >= 0.95:
                rospy.logwarn("ignoring large angle check engaged!")
                speedup_path = self.speedup(path)
                execute_result = group.execute(speedup_path, wait=True)
                t.join()
                self.cleanup_node()
                return execute_result
            
            if fraction >= 0.90:
                # check if shoulder_base rotates more then 100degree or 1.745rad
                num_point = len(path.joint_trajectory.points)
                large_turning = False
                for i in range(num_point):
                    if i > 0:
                        check_joint_0 = abs(path.joint_trajectory.points[i].positions[0] - path.joint_trajectory.points[0].positions[0])
                        check_joint_2 = path.joint_trajectory.points[i].positions[2]
                        check_joint_3 = abs(path.joint_trajectory.points[i].positions[3] - path.joint_trajectory.points[0].positions[3])
                        if ((check_joint_0 > 1.745) or (check_joint_2 <= 0) or (check_joint_3 > 2)):
                            rospy.logwarn("Large turning is predicted, retrying")
                            large_turning = True
                            break
                
                if not large_turning:
                    speedup_path = self.speedup(path)
                    execute_result = group.execute(speedup_path, wait=True)
                    stop_marker = True
                    t.join()
                    self.cleanup_node()
                    return execute_result
        return False


    def rotate_joint(self, target_quatenion, manipulator = "parallel_gripper_manipulator"):
        group = eval("self." + manipulator)
        joint = group.get_current_pose().pose.position
        target_pose = geometry_msgs.msg.Pose()
        target_pose.position = joint
        target_pose.orientation.x = target_quatenion[0]
        target_pose.orientation.y = target_quatenion[1]
        target_pose.orientation.z = target_quatenion[2]
        target_pose.orientation.w = target_quatenion[3]
        group.set_pose_target(target_pose)
        go_result = group.go(wait=True)
        group.stop()
        return go_result

    def get_coor(self, manipulator = "parallel_gripper_manipulator", print_output = True):
        group = eval("self." + current_gripper)
        joint = group.get_current_pose()
        if print_output:
            print("=========== pose")
            print(joint)
            print(joint.pose.position.x, joint.pose.position.y, joint.pose.position.z, joint.pose.orientation.x, joint.pose.orientation.y, joint.pose.orientation.z, joint.pose.orientation.w)
        return joint
    
    def get_joint(self, manipulator = "parallel_gripper_manipulator"):
        group = eval("self." + current_gripper)
        joint = group.get_current_joint_values()
        print("=========== joint")
        print(joint)
        return joint

    def move_with_cartesian(self, pose_goal):
        group = eval("self." + current_gripper)
        group.set_pose_target(pose_goal)
        go_result = group.go(wait=True)
        group.stop()
        return go_result

############################################################################################# pre plan motion
def placing(starting_location, placing_coor, extra_zdown, pigeonhole_processing):
    target_markers.deleteAllMarkers()
    marker_pose = Pose()
    input_list = [placing_coor[0], placing_coor[1], placing_coor[2] - 0.1 - extra_zdown, placing_coor[3], placing_coor[4], placing_coor[5], placing_coor[6]]
    marker_pose = list_to_pose(input_list)
    target_markers.publishAxis(marker_pose, 0.15, 0.01, None)

    if control.plan_and_move_new(placing_coor, collisions_bool=True, gripper=current_gripper) or control.plan_and_move_new(placing_coor, collisions_bool=True, gripper=current_gripper, ignore_large_angle_checking= True):
        if control.plan_cartesian_path_new("z", "down", 0.1 + extra_zdown, True, current_gripper) or control.plan_cartesian_path_new("z", "down", 0.1 + extra_zdown, ignore_large_angle_checking= True, collisions_bool= True, gripper = current_gripper):
            control.gripper_control("release")
            if control.plan_cartesian_path_new("z", "up", 0.1 + extra_zdown, True, current_gripper) or control.plan_cartesian_path_new("z", "up", 0.1 + extra_zdown, ignore_large_angle_checking= True, collisions_bool= True, gripper = current_gripper):
                if pigeonhole_processing:
                    rospy.loginfo("placing at wetbay or drybay, z up to 1.35m")
                    z_increment = 1.35 - control.get_coor().pose.position.z 
                    if control.plan_cartesian_path_new("z", "up", z_increment, True, current_gripper) or control.plan_cartesian_path_new("z", "up", z_increment, ignore_large_angle_checking=True, collisions_bool= True, gripper= current_gripper):
                        print("ok")
                    else:
                        rospy.logerr("something is wrong during z up 1.35m")
                else:
                    rospy.logwarn("no pigeonhole is provided")
                if starting_location == "wetbay":
                    control.wetbay_homing()
                elif starting_location == "drybay":
                    control.drybay_homing()
                elif starting_location == "pt1":
                    control.pt1_homing() 
                elif starting_location == "pt1_instrument_pap":
                    control.pt1_instrument_pap_homing()   
                elif starting_location == "pt2":
                    control.pt2_homing() 
                rospy.loginfo("task completed successfully!")
                return True
            else:
                rospy.logwarn("error while planning z place up")
                rospy.logerr("Retrying threshold reached, returning fail")
                return False
        else:
            rospy.logwarn("error while planning z place down")
            rospy.logerr("Retrying threshold reached, returning fail")
            return False
    else:
        rospy.logwarn("error while moving to placing coor")
        rospy.logerr("Retrying threshold reached, returning fail")
        return False

def pt1_fail_pick_release(coordinate, err = False):
    if control.pt1_instrument_pap_homing():
        if err:
            if control.pt1_fail_pick_release_homing():
                control.gripper_control("release")
                control.pt1_instrument_pap_homing()
                return True
        else:
            if control.plan_and_move(coordinate, True, current_gripper):
                control.gripper_control("release")
                control.pt1_instrument_pap_homing()
                return True
    return False

def BACKUP_drybay_scissor_brac_side_pushing(motion_part, which_bracket, bracket_coor, tray_coor, pigeonhole_processing):
    """motion 1 - place and push bracket; motion 2 - go to side pushing location 1; motion 3 - go to side pushing end location"""
    if motion_part == 1:
        """put down bracket and push it down"""
        temp_coor = [bracket_coor[0]-0.05, bracket_coor[1], bracket_coor[2]+0.11, bracket_coor[3], bracket_coor[4], bracket_coor[5], bracket_coor[6]]
        if control.plan_and_move_new(temp_coor, collisions_bool= True, gripper= current_gripper) or control.plan_and_move_new(temp_coor, collisions_bool= True, gripper= current_gripper, ignore_large_angle_checking= True):
            if control.plan_cartesian_path_new("z", "down", 0.1, collisions_bool= True, gripper= current_gripper) or control.plan_cartesian_path_new("z", "down", 0.1, collisions_bool= True, gripper= current_gripper, ignore_large_angle_checking=True):
                return True
                if control.plan_cartesian_path_new("x", "up", 0.05, collisions_bool= True, gripper= current_gripper) or control.plan_cartesian_path_new("x", "up", 0.05, collisions_bool= True, gripper= current_gripper, ignore_large_angle_checking=True):
                    control.gripper_control("release")
                    if control.plan_cartesian_path_new("z", "up", 0.15, collisions_bool= True, gripper= current_gripper) or control.plan_cartesian_path_new("z", "up", 0.15, collisions_bool= True, gripper= current_gripper, ignore_large_angle_checking=True):
                        control.gripper_control("grip")
                    else:
                        rospy.logerr("error during z up")
                        return False
                else:
                    rospy.logerr("error during x up")
                    return False
            else:
                rospy.logerr("error during z down")
                return False
        else:
            rospy.logerr("error during moving")
            return False
        if which_bracket == 1:
            temp_coor = [bracket_coor[0]+0.1, bracket_coor[1]-0.01, bracket_coor[2]-0.01, 1,0,0,0]
            # temp_coor2 = [temp_coor[0]-0.2, temp_coor[1]+0.03, temp_coor[2], temp_coor[3], temp_coor[4], temp_coor[5], temp_coor[6]]
        elif which_bracket == 2:
            temp_coor = [bracket_coor[0]+0.1, bracket_coor[1]+0.01, bracket_coor[2]-0.01, 0, 1, 0,0]
            # temp_coor2 = [temp_coor[0]-0.2, temp_coor[1]-0.03, temp_coor[2], temp_coor[3], temp_coor[4], temp_coor[5], temp_coor[6]]
        if control.plan_and_move_new(temp_coor, collisions_bool= True, gripper= current_gripper) or control.plan_and_move_new(temp_coor, collisions_bool= True, gripper= current_gripper, ignore_large_angle_checking= True):
            if control.plan_cartesian_path_new("x", "down", 0.2, collisions_bool= True, gripper= current_gripper) or control.plan_cartesian_path_new("x", "down", 0.2, collisions_bool= True, gripper= current_gripper, ignore_large_angle_checking= True):
            # if control.plan_and_move_new(temp_coor2, collisions_bool= True, gripper= current_gripper) or control.plan_and_move_new(temp_coor2, collisions_bool= True, gripper= current_gripper, ignore_large_angle_checking= True):
                control.drybay_homing()
                return True
            else:
                rospy.logerr("error during z down")
                return False
        else:
            rospy.logerr("error during moving")
            return False
    elif motion_part == 2:
        """push the bracket to the side"""
        temp_coor = [bracket_coor[0], bracket_coor[1], bracket_coor[2], bracket_coor[3], bracket_coor[4], bracket_coor[5], bracket_coor[6]]
        if pigeonhole_processing <= 3:
            temp_coor[2] = 1.175
        elif pigeonhole_processing > 3:
            temp_coor[2] = 0.845 #drag gripper 0.845
        if control.plan_and_move_new(temp_coor, collisions_bool= True, gripper= current_gripper) or control.plan_and_move_new(temp_coor, collisions_bool= True, gripper= current_gripper, ignore_large_angle_checking= True):
            if control.plan_cartesian_path_new("z", "down", 0.1, collisions_bool= True, gripper= current_gripper) or control.plan_cartesian_path_new("z", "down", 0.1, collisions_bool= True, gripper= current_gripper, ignore_large_angle_checking=True):
                pass
            else:
                rospy.logerr("error during z down")
                return False
        else:
            rospy.logerr("error during moving")
            return False
        if which_bracket == 1:
            temp_coor = [tray_coor[0] + 0.031, tray_coor[1] - 0.286, temp_coor[2]-0.1, 0, 1, 0, 0] #x was 0.031, y was 0.286
        elif which_bracket == 2:
            temp_coor = [tray_coor[0] + 0.031, tray_coor[1] - 0.156, temp_coor[2]-0.1, 0, 1, 0, 0]
        control.move_with_cartesian(temp_coor)
        if control.plan_cartesian_path_new("z", "up", 0.1, collisions_bool= True, gripper= current_gripper) or control.plan_cartesian_path_new("z", "up", 0.1, collisions_bool= True, gripper= current_gripper, ignore_large_angle_checking=True):
            rospy.loginfo("z up to 1.35m")
            z_increment = 1.35 - control.get_coor().pose.position.z 
            if control.plan_cartesian_path_new("z", "up", z_increment, True, current_gripper) or control.plan_cartesian_path_new("z", "up", z_increment, ignore_large_angle_checking=True, collisions_bool= True, gripper= current_gripper):
                control.drybay_homing()
                return True
            else:
                rospy.logerr("error during z up tp 1.35m")
                return False
        else:
            rospy.logerr("error during z down")
            return False

def drybay_scissor_brac_side_pushing(motion_part, which_bracket, bracket_coor, tray_coor, pigeonhole_processing):
    """motion 1 - place and push bracket; motion 2 - go to side pushing location 1; motion 3 - go to side pushing end location"""
    if motion_part == 1:
        """put down bracket and push it down"""
        temp_coor = [bracket_coor[0]-0.05, bracket_coor[1], bracket_coor[2]+0.11, bracket_coor[3], bracket_coor[4], bracket_coor[5], bracket_coor[6]]
        if control.plan_and_move_new(temp_coor, collisions_bool= True, gripper= current_gripper) or control.plan_and_move_new(temp_coor, collisions_bool= True, gripper= current_gripper, ignore_large_angle_checking= True):
            if control.plan_cartesian_path_new("z", "down", 0.1, collisions_bool= True, gripper= current_gripper) or control.plan_cartesian_path_new("z", "down", 0.1, collisions_bool= True, gripper= current_gripper, ignore_large_angle_checking=True):
                if which_bracket == 1:
                    if control.plan_cartesian_path_new("x", "up", 0.05, collisions_bool= True, gripper= current_gripper) or control.plan_cartesian_path_new("x", "up", 0.05, collisions_bool= True, gripper= current_gripper, ignore_large_angle_checking=True):
                        pass
                elif which_bracket == 2:
                    if control.plan_cartesian_path_new("x", "down", 0.03, collisions_bool= True, gripper= current_gripper) or control.plan_cartesian_path_new("x", "down", 0.03, collisions_bool= True, gripper= current_gripper, ignore_large_angle_checking=True):
                        pass
                control.gripper_control("release")
                if control.plan_cartesian_path_new("z", "up", 0.15, collisions_bool= True, gripper= current_gripper) or control.plan_cartesian_path_new("z", "up", 0.15, collisions_bool= True, gripper= current_gripper, ignore_large_angle_checking=True):
                    control.gripper_control("grip")
                else:
                    rospy.logerr("error during z up")
                    return False
            else:
                rospy.logerr("error during z down")
                return False
        else:
            rospy.logerr("error during moving")
            return False

        current_coor = control.get_coor(current_gripper, print_output=False).pose.position
        if which_bracket == 1:
            temp_coor = [current_coor.x+0.1, current_coor.y-0.01, bracket_coor[2]-0.01, 1,0,0,0]
        elif which_bracket == 2:
            temp_coor = [current_coor.x-0.1, current_coor.y+0.02, bracket_coor[2]-0.01, -0.087, -0.996, 0, 0]
        if control.plan_and_move_new(temp_coor, collisions_bool= True, gripper= current_gripper) or control.plan_and_move_new(temp_coor, collisions_bool= True, gripper= current_gripper, ignore_large_angle_checking= True):
            if which_bracket == 1:
                if control.plan_cartesian_path_new("x", "down", 0.25, collisions_bool= True, gripper= current_gripper) or control.plan_cartesian_path_new("x", "down", 0.2, collisions_bool= True, gripper= current_gripper, ignore_large_angle_checking= True):
                    pass
            elif which_bracket == 2:
                if control.plan_cartesian_path_new("x", "up", 0.25, collisions_bool= True, gripper= current_gripper) or control.plan_cartesian_path_new("x", "down", 0.2, collisions_bool= True, gripper= current_gripper, ignore_large_angle_checking= True):
                    pass
            control.drybay_homing()
            return True
        else:
            rospy.logerr("error during moving")
            return False
    elif motion_part == 2:
        """push the bracket to the side"""
        temp_coor = [bracket_coor[0], bracket_coor[1], bracket_coor[2], bracket_coor[3], bracket_coor[4], bracket_coor[5], bracket_coor[6]]
        if pigeonhole_processing <= 3:
            temp_coor[2] = 1.175
        elif pigeonhole_processing > 3:
            temp_coor[2] = 0.845 #drag gripper 0.845
        if control.plan_and_move_new(temp_coor, collisions_bool= True, gripper= current_gripper) or control.plan_and_move_new(temp_coor, collisions_bool= True, gripper= current_gripper, ignore_large_angle_checking= True):
            if control.plan_cartesian_path_new("z", "down", 0.1, collisions_bool= True, gripper= current_gripper) or control.plan_cartesian_path_new("z", "down", 0.1, collisions_bool= True, gripper= current_gripper, ignore_large_angle_checking=True):
                pass
            else:
                rospy.logerr("error during z down")
                return False
        else:
            rospy.logerr("error during moving")
            return False
        if which_bracket == 1:
            temp_coor = [tray_coor[0] + 0.031, tray_coor[1] - 0.286, temp_coor[2]-0.1, 0, 1, 0, 0] #x was 0.031, y was 0.286
        elif which_bracket == 2:
            temp_coor = [tray_coor[0] + 0.031, tray_coor[1] - 0.156, temp_coor[2]-0.1, 0, 1, 0, 0]
        control.move_with_cartesian(temp_coor)
        if control.plan_cartesian_path_new("z", "up", 0.1, collisions_bool= True, gripper= current_gripper) or control.plan_cartesian_path_new("z", "up", 0.1, collisions_bool= True, gripper= current_gripper, ignore_large_angle_checking=True):
            rospy.loginfo("z up to 1.35m")
            z_increment = 1.35 - control.get_coor().pose.position.z 
            if control.plan_cartesian_path_new("z", "up", z_increment, True, current_gripper) or control.plan_cartesian_path_new("z", "up", z_increment, ignore_large_angle_checking=True, collisions_bool= True, gripper= current_gripper):
                control.drybay_homing()
                return True
            else:
                rospy.logerr("error during z up tp 1.35m")
                return False
        else:
            rospy.logerr("error during z down")
            return False

def drybay_forcep_bracket_placing(motion_part, bracket_coor, tray_coor, pigeonhole_processing):
    """motion 1 - place and push bracket; motion 2 - go to side pushing location 1; motion 3 - go to side pushing end location"""
    if motion_part == 1:
        """put down bracket and push it down"""
        temp_coor = [bracket_coor[0]-0.05, bracket_coor[1], bracket_coor[2]+0.11, bracket_coor[3], bracket_coor[4], bracket_coor[5], bracket_coor[6]]
        if control.plan_and_move_new(temp_coor, collisions_bool= True, gripper= current_gripper) or control.plan_and_move_new(temp_coor, collisions_bool= True, gripper= current_gripper, ignore_large_angle_checking= True):
            if control.plan_cartesian_path_new("z", "down", 0.1, collisions_bool= True, gripper= current_gripper) or control.plan_cartesian_path_new("z", "down", 0.1, collisions_bool= True, gripper= current_gripper, ignore_large_angle_checking=True):
                if control.plan_cartesian_path_new("x", "up", 0.05, collisions_bool= True, gripper= current_gripper) or control.plan_cartesian_path_new("x", "up", 0.05, collisions_bool= True, gripper= current_gripper, ignore_large_angle_checking=True):
                    control.gripper_control("release")
                    if control.plan_cartesian_path_new("z", "up", 0.15, collisions_bool= True, gripper= current_gripper) or control.plan_cartesian_path_new("z", "up", 0.15, collisions_bool= True, gripper= current_gripper, ignore_large_angle_checking=True):
                        control.gripper_control("grip")
                        current_coor = control.get_coor(current_gripper, print_output=False).pose.position
                        # temp_coor = [current_coor.x+0.1, current_coor.y-0.01, bracket_coor[2]-0.01, 1,0,0,0]
                        temp_coor = [current_coor.x+0.1, current_coor.y-0.01, bracket_coor[2]-0.01, 0.985, -0.174, 0, 0]
                        if control.plan_and_move_new(temp_coor, collisions_bool= True, gripper= current_gripper) or control.plan_and_move_new(temp_coor, collisions_bool= True, gripper= current_gripper, ignore_large_angle_checking= True):
                            if control.plan_cartesian_path_new("x", "down", 0.25, collisions_bool= True, gripper= current_gripper) or control.plan_cartesian_path_new("x", "down", 0.2, collisions_bool= True, gripper= current_gripper, ignore_large_angle_checking= True):
                                rospy.loginfo("z up to 1.25m")
                                z_increment = 1.25 - control.get_coor().pose.position.z 
                                if control.plan_cartesian_path_new("z", "up", z_increment, True, current_gripper) or control.plan_cartesian_path_new("z", "up", z_increment, ignore_large_angle_checking=True, collisions_bool= True, gripper= current_gripper):
                                    control.drybay_homing()
                                    return True
                                else:
                                    rospy.logerr("error during z up 1.25m")
                                    return False
                            else:
                                rospy.logerr("error during x down")
                                return False
                        else:
                            rospy.logerr("error during moving")
                            return False
                    else:
                        rospy.logerr("error during z up")
                        return False
                else:
                    rospy.logerr("error during x up")
                    return False
            else:
                rospy.logerr("error during z down")
                return False
        else:
            rospy.logerr("error during moving")
            return False

    elif motion_part == 2:
        """push the bracket to the side"""
        temp_coor = [bracket_coor[0], bracket_coor[1], tray_coor[2], bracket_coor[3], bracket_coor[4], bracket_coor[5], bracket_coor[6]]
        if control.plan_and_move_new(temp_coor, collisions_bool= True, gripper= current_gripper) or control.plan_and_move_new(temp_coor, collisions_bool= True, gripper= current_gripper, ignore_large_angle_checking= True):
            if control.plan_cartesian_path_new("z", "down", 0.1, collisions_bool= True, gripper= current_gripper) or control.plan_cartesian_path_new("z", "down", 0.1, collisions_bool= True, gripper= current_gripper, ignore_large_angle_checking=True):
                temp_coor = [tray_coor[0] + 0.031, tray_coor[1] - 0.286, temp_coor[2]-0.1, 0, 1, 0, 0] #x was 0.031, y was 0.286
                control.move_with_cartesian(temp_coor)
                if control.plan_cartesian_path_new("z", "up", 0.1, collisions_bool= True, gripper= current_gripper) or control.plan_cartesian_path_new("z", "up", 0.1, collisions_bool= True, gripper= current_gripper, ignore_large_angle_checking=True):
                    rospy.loginfo("z up to 1.25m")
                    z_increment = 1.25 - control.get_coor().pose.position.z 
                    if control.plan_cartesian_path_new("z", "up", z_increment, True, current_gripper) or control.plan_cartesian_path_new("z", "up", z_increment, ignore_large_angle_checking=True, collisions_bool= True, gripper= current_gripper):
                        control.drybay_homing()
                        return True
                    else:
                        rospy.logerr("error during z up tp 1.25m")
                        return False
                else:
                    rospy.logerr("error during z down")
                    return False
            else:
                rospy.logerr("error during z down")
                return False
        else:
            rospy.logerr("error during moving")
            return False
            
def drybay_mix_bracket_placing(bracket_coor, tray_coor, pigeonhole_processing):
    """put down bracket and push it down"""
    temp_coor = [bracket_coor[0]-0.05, bracket_coor[1], bracket_coor[2]+0.11, bracket_coor[3], bracket_coor[4], bracket_coor[5], bracket_coor[6]]
    if control.plan_and_move_new(temp_coor, collisions_bool= True, gripper= current_gripper) or control.plan_and_move_new(temp_coor, collisions_bool= True, gripper= current_gripper, ignore_large_angle_checking= True):
        if control.plan_cartesian_path_new("z", "down", 0.1, collisions_bool= True, gripper= current_gripper) or control.plan_cartesian_path_new("z", "down", 0.1, collisions_bool= True, gripper= current_gripper, ignore_large_angle_checking=True):
            if control.plan_cartesian_path_new("x", "down", 0.03, collisions_bool= True, gripper= current_gripper) or control.plan_cartesian_path_new("x", "down", 0.03, collisions_bool= True, gripper= current_gripper, ignore_large_angle_checking=True):
                control.gripper_control("release")
                if control.plan_cartesian_path_new("z", "up", 0.15, collisions_bool= True, gripper= current_gripper) or control.plan_cartesian_path_new("z", "up", 0.15, collisions_bool= True, gripper= current_gripper, ignore_large_angle_checking=True):
                    control.gripper_control("grip")
                    current_coor = control.get_coor(current_gripper, print_output=False).pose.position
                    temp_coor = [current_coor.x-0.1, current_coor.y+0.02, bracket_coor[2]-0.01, -0.087, -0.996, 0, 0]
                    if control.plan_and_move_new(temp_coor, collisions_bool= True, gripper= current_gripper) or control.plan_and_move_new(temp_coor, collisions_bool= True, gripper= current_gripper, ignore_large_angle_checking= True):
                        if control.plan_cartesian_path_new("x", "up", 0.25, collisions_bool= True, gripper= current_gripper) or control.plan_cartesian_path_new("x", "down", 0.2, collisions_bool= True, gripper= current_gripper, ignore_large_angle_checking= True):
                            rospy.loginfo("z up to 1.25m")
                            z_increment = 1.25 - control.get_coor().pose.position.z 
                            if control.plan_cartesian_path_new("z", "up", z_increment, True, current_gripper) or control.plan_cartesian_path_new("z", "up", z_increment, ignore_large_angle_checking=True, collisions_bool= True, gripper= current_gripper):
                                control.drybay_homing()
                                return True
                            else:
                                rospy.logerr("error during z up 1.25m")
                                return False
                        else:
                            rospy.logerr("error during x up")
                            return False
                    else:
                        rospy.logerr("error during moving")
                        return False
                else:
                    rospy.logerr("error during z up")
                    return False
            else:
                rospy.logerr("error during x down")
                return False
        else:
            rospy.logerr("error during z down")
            return False
    else:
        rospy.logerr("error during moving")
        return False
        

def picking(starting_location, picking_coor, extra_zdown, pigeonhole_processing):
    control.gripper_control("release")
    target_markers.deleteAllMarkers()
    marker_pose = Pose()
    input_list = [picking_coor[0], picking_coor[1], picking_coor[2] - 0.1 - extra_zdown, picking_coor[3], picking_coor[4], picking_coor[5], picking_coor[6]]
    marker_pose = list_to_pose(input_list)
    target_markers.publishAxis(marker_pose, 0.15, 0.01, None)
    if control.plan_and_move_new(picking_coor, collisions_bool=True, gripper=current_gripper) or control.plan_and_move_new(picking_coor, collisions_bool=True, gripper=current_gripper, ignore_large_angle_checking= True):
        if control.plan_cartesian_path_new("z", "down", 0.1 + extra_zdown, True, current_gripper) or control.plan_cartesian_path_new("z", "down", 0.1 + extra_zdown, ignore_large_angle_checking= True, collisions_bool= True, gripper = current_gripper):
            control.gripper_control("grip")
            sleep(1)
            if control.plan_cartesian_path_new("z", "up", 0.1 + extra_zdown, True, current_gripper) or control.plan_cartesian_path_new("z", "up", 0.1 + extra_zdown, ignore_large_angle_checking= True, collisions_bool= True, gripper = current_gripper):
                if pigeonhole_processing:
                    rospy.loginfo("placing at wetbay or drybay, z up to 1.35m")
                    z_increment = 1.35 - control.get_coor().pose.position.z 
                    if control.plan_cartesian_path_new("z", "up", z_increment, True, current_gripper) or control.plan_cartesian_path_new("z", "up", z_increment, ignore_large_angle_checking=True, collisions_bool= True, gripper= current_gripper):
                        print("ok")
                    else:
                        rospy.logerr("something is wrong during z up 1.35m")
                else:
                    rospy.logwarn("no pigeonhole is provided")
                if starting_location == "wetbay":
                    control.wetbay_homing()
                elif starting_location == "drybay":
                    control.drybay_homing()
                elif starting_location == "pt1":
                    control.pt1_homing() 
                elif starting_location == "pt1_instrument_pap":
                    control.pt1_instrument_pap_homing()   
                elif starting_location == "pt2":
                    control.pt2_homing() 
                rospy.loginfo("task completed successfully!")
                return True
            else:
                rospy.logwarn("error while planning z place up")
                rospy.logerr("Retrying threshold reached, returning fail")
                return False
        else:
            rospy.logwarn("error while planning z place down")
            rospy.logerr("Retrying threshold reached, returning fail")
            return False
    else:
        rospy.logwarn("error while moving to placing coor")
        rospy.logerr("Retrying threshold reached, returning fail")
        return False

def wetbay_picking(picking_coor, extra_zdown, item, pigeonhole_processing):
    control.gripper_control("release")
    target_markers.deleteAllMarkers()
    marker_pose = Pose()
    input_list = [picking_coor[0], picking_coor[1], picking_coor[2] - 0.1 - extra_zdown, picking_coor[3], picking_coor[4], picking_coor[5], picking_coor[6]]
    marker_pose = list_to_pose(input_list)
    target_markers.publishAxis(marker_pose, 0.15, 0.01, None)
    print(picking_coor)
    if control.plan_and_move_new(picking_coor, collisions_bool= True, gripper = current_gripper) or control.plan_and_move_new(picking_coor, collisions_bool= True, gripper = current_gripper, ignore_large_angle_checking= True):
        print("debug, plan and move new completed")
        if control.plan_cartesian_path_new("z", "down", 0.1 + extra_zdown,collisions_bool= True,gripper= current_gripper) or control.plan_cartesian_path_new("z", "down", 0.1 + extra_zdown,collisions_bool= True,gripper= current_gripper, ignore_large_angle_checking= True):
            gripper_opening = control.gripper_control("grip")
            sleep(1)
            if gripper_opening == 0 and item != "wetbay_gallipot":  #disable opening check for gallipot
                #gripper didnt picked anything
                rospy.logwarn("gripper opening is 0, might not picked anything, retrying")
                control.gripper_control("release")
                control.homing()
                return False
            #######specific process for specific item
            if item == "wetbay_tray":
                if gripper_opening > 10:
                    rospy.logwarn("did not successfully grip tray center")
                    trial = 0
                    while(gripper_opening > 10) and trial < 3:
                        control.gripper_control("release")
                        if(pigeonhole_processing == 3) or (pigeonhole_processing == 6):
                            if control.plan_cartesian_path_new("y", "down", 0.01 + extra_zdown,collisions_bool= True,gripper= current_gripper) or control.plan_cartesian_path_new("y", "down", 0.01 + extra_zdown,collisions_bool= True,gripper= current_gripper, ignore_large_angle_checking= True):
                                gripper_opening = control.gripper_control("grip")
                        elif(pigeonhole_processing == 1) or (pigeonhole_processing == 4):
                            if control.plan_cartesian_path_new("y", "up", 0.01 + extra_zdown,collisions_bool= True,gripper= current_gripper) or control.plan_cartesian_path_new("y", "up", 0.01 + extra_zdown,collisions_bool= True,gripper= current_gripper, ignore_large_angle_checking= True):
                                gripper_opening = control.gripper_control("grip")
                        trial += 1
                control.plan_cartesian_path_new("x", "up", 0.02, True, current_gripper)
            elif item == "wetbay_bracket":
                if not control.plan_cartesian_path_new("z", "up", 0.015, True):
                    control.plan_cartesian_path_new("z", "up", 0.015, ignore_large_angle_checking= True, collisions_bool= True, gripper = current_gripper)
                if not control.plan_cartesian_path_new("x", "up", 0.05, True):
                    control.plan_cartesian_path_new("x", "up", 0.05, ignore_large_angle_checking= True, collisions_bool= True, gripper = current_gripper)
                    
            #######specific process for specific item ends    
            if control.plan_cartesian_path_new("z", "up", 0.101 + extra_zdown, True, current_gripper) or control.plan_cartesian_path_new("z", "up", 0.1 + extra_zdown, ignore_large_angle_checking= True, collisions_bool= True, gripper = current_gripper):
                # move z uptil 1.35
                rospy.loginfo("z up to 1.35m")
                z_increment = 1.35 - control.get_coor().pose.position.z 
                if not control.plan_cartesian_path_new("z", "up", z_increment, True, current_gripper):
                    control.plan_cartesian_path_new("z", "up", z_increment, ignore_large_angle_checking=True, collisions_bool= True, gripper= current_gripper)
                if item == "wetbay_bracket":
                    control.pt2_homing()
                elif item == "wetbay_kidney_dish":
                    control.pt1_homing()
                elif item == "wetbay_gallipot":
                    control.drybay_homing()
                else:
                    control.wetbay_homing()
                rospy.loginfo("task completed successfully!")
                target_markers.deleteAllMarkers()
                return True
            else:
                rospy.logwarn("error while planning z pick up, retrying...")
                rospy.logerr("Retrying threshold reached, returning fail")
                control.gripper_control("release")
                control.homing()
                return False
        else:
            rospy.logwarn("error while planning z pick down")
            rospy.logerr("Retrying threshold reached, returning fail")
            control.homing()
            return False
    else:
        rospy.logwarn("error while planning go to picking coor")
        return False

def drybay_picking(picking_coor, extra_zdown, item, starting_location):
    control.gripper_control("release")
    target_markers.deleteAllMarkers()
    marker_pose = Pose()
    input_list = [picking_coor[0], picking_coor[1], picking_coor[2] - 0.1 - extra_zdown, picking_coor[3], picking_coor[4], picking_coor[5], picking_coor[6]]
    marker_pose = list_to_pose(input_list)
    target_markers.publishAxis(marker_pose, 0.15, 0.01, None)
    print(picking_coor)
    if control.plan_and_move_new(picking_coor, collisions_bool= True, gripper = current_gripper) or control.plan_and_move_new(picking_coor, collisions_bool= True, gripper = current_gripper, ignore_large_angle_checking= True):
        if control.plan_cartesian_path_new("z", "down", 0.1 + extra_zdown,collisions_bool= True,gripper= current_gripper) or control.plan_cartesian_path_new("z", "down", 0.1 + extra_zdown,collisions_bool= True,gripper= current_gripper, ignore_large_angle_checking= True):
            gripper_opening = control.gripper_control("grip")
            sleep(1)
            if gripper_opening == 0 and (item != "drybay_gallipot" or item != "drybay_indicator"):  #disable opening check for gallipot
                #gripper didnt picked anything
                rospy.logwarn("gripper opening is 0, might not picked anything, retrying")
                control.gripper_control("release")
                eval("control." + starting_location + "_homing()")
                return False
            #######specific process for specific item
            if item == "WBT":
                z_increment = 1.25 - control.get_coor().pose.position.z 
                if not control.plan_cartesian_path_new("z", "up", z_increment, True, current_gripper):
                    control.plan_cartesian_path_new("z", "up", z_increment, ignore_large_angle_checking=True, collisions_bool= True, gripper= current_gripper)
            #######specific process for specific item ends
            if control.plan_cartesian_path_new("z", "up", 0.1 + extra_zdown, True, current_gripper) or control.plan_cartesian_path_new("z", "up", 0.1 + extra_zdown, ignore_large_angle_checking= True, collisions_bool= True, gripper = current_gripper):
                eval("control." + starting_location + "_homing()")
                rospy.loginfo("task completed successfully!")
                target_markers.deleteAllMarkers()
                return True
            else:
                rospy.logwarn("error while planning z pick up, retrying...")
                rospy.logerr("Retrying threshold reached, returning fail")
                control.gripper_control("release")
                eval("control." + starting_location + "_homing()")
                return False
        else:
            rospy.logwarn("error while planning z pick down")
            rospy.logerr("Retrying threshold reached, returning fail")
            eval("control." + starting_location + "_homing()")
            return False
    else:
        rospy.logwarn("error while planning go to picking coor")
        return False

def drag_instrument(instrument_coor, drag_direction):
    global picking_joint
    target_markers.deleteAllMarkers()
    marker_pose = Pose()
    input_list = [instrument_coor[0], instrument_coor[1], instrument_coor[2] - 0.02, instrument_coor[3], instrument_coor[4], instrument_coor[5], instrument_coor[6]]
    marker_pose = list_to_pose(input_list)
    target_markers.publishAxis(marker_pose, 0.15, 0.01, None)
    print(instrument_coor)
    if control.plan_and_move_new(instrument_coor, collisions_bool= True, gripper= current_gripper) or control.plan_and_move_new(instrument_coor, collisions_bool= True, gripper= current_gripper, ignore_large_angle_checking= True):
        if control.plan_cartesian_path_new("z", "down", 0.05, collisions_bool= True, gripper= current_gripper) or control.plan_cartesian_path_new("z", "down", 0.05, collisions_bool= True, gripper= current_gripper, ignore_large_angle_checking= True):
            if z_down_touch():
                if drag_direction == "top":
                    if control.plan_cartesian_path_new("y", "down", 0.03, collisions_bool= True, gripper= current_gripper) or control.plan_cartesian_path_new("y", "down", 0.03, collisions_bool= True, gripper= current_gripper, ignore_large_angle_checking= True):
                        rospy.loginfo("task completed successfully!")
                        target_markers.deleteAllMarkers()
                        if control.plan_cartesian_path_new("z", "up", 0.1, collisions_bool= True, gripper= current_gripper) or control.plan_cartesian_path_new("z", "up", 0.1, collisions_bool= True, gripper= current_gripper, ignore_large_angle_checking= True):
                            rospy.loginfo("task completed successfully!")
                            control.pt1_instrument_pap_homing()
                            return True
                        else:
                            rospy.logwarn("error while planning z up...")
                            return False
                    else:
                        rospy.logwarn("error while planning y down...")
                        return False
                elif drag_direction == "btm":
                    if control.plan_cartesian_path_new("y", "up", 0.03, collisions_bool= True, gripper= current_gripper) or control.plan_cartesian_path_new("y", "up", 0.03, collisions_bool= True, gripper= current_gripper, ignore_large_angle_checking= True):
                        rospy.loginfo("task completed successfully!")
                        target_markers.deleteAllMarkers()
                        if control.plan_cartesian_path_new("z", "up", 0.1, collisions_bool= True, gripper= current_gripper) or control.plan_cartesian_path_new("z", "up", 0.1, collisions_bool= True, gripper= current_gripper, ignore_large_angle_checking= True):
                            rospy.loginfo("task completed successfully!")
                            control.pt1_instrument_pap_homing()
                            return True
                        else:
                            rospy.logwarn("error while planning z up...")
                            return False
                    else:
                        rospy.logwarn("error while planning y down...")
                        return False
                elif drag_direction == "left":
                    if control.plan_cartesian_path_new("x", "up", 0.05, collisions_bool= True, gripper= current_gripper) or control.plan_cartesian_path_new("x", "up", 0.05, collisions_bool= True, gripper= current_gripper, ignore_large_angle_checking= True):
                        rospy.loginfo("task completed successfully!")
                        target_markers.deleteAllMarkers()
                        if control.plan_cartesian_path_new("z", "up", 0.1, collisions_bool= True, gripper= current_gripper) or control.plan_cartesian_path_new("z", "up", 0.1, collisions_bool= True, gripper= current_gripper, ignore_large_angle_checking= True):
                            rospy.loginfo("task completed successfully!")
                            control.pt1_instrument_pap_homing()
                            return True
                        else:
                            rospy.logwarn("error while planning z up...")
                            return False
                    else:
                        rospy.logwarn("error while planning y down...")
                        return False
                elif drag_direction == "right":
                    if control.plan_cartesian_path_new("x", "down", 0.05, collisions_bool= True, gripper= current_gripper) or control.plan_cartesian_path_new("x", "down", 0.05, collisions_bool= True, gripper= current_gripper, ignore_large_angle_checking= True):
                        rospy.loginfo("task completed successfully!")
                        target_markers.deleteAllMarkers()
                        if control.plan_cartesian_path_new("z", "up", 0.1, collisions_bool= True, gripper= current_gripper) or control.plan_cartesian_path_new("z", "up", 0.1, collisions_bool= True, gripper= current_gripper, ignore_large_angle_checking= True):
                            rospy.loginfo("task completed successfully!")
                            control.pt1_instrument_pap_homing()
                            return True
                        else:
                            rospy.logwarn("error while planning z up...")
                            return False
                    else:
                        rospy.logwarn("error while planning y down...")
                        return False
                elif drag_direction == "faulty_ocm_instrument":
                    y_diff = abs(control.get_coor(current_gripper).pose.position.y - 0.46)
                    if control.plan_cartesian_path_new("y", "down", y_diff, collisions_bool= True, gripper= current_gripper) or control.plan_cartesian_path_new("y", "down", y_diff, collisions_bool= True, gripper= current_gripper, ignore_large_angle_checking= True):
                        rospy.loginfo("task completed successfully!")
                        target_markers.deleteAllMarkers()
                        if control.plan_cartesian_path_new("z", "up", 0.1, collisions_bool= True, gripper= current_gripper) or control.plan_cartesian_path_new("z", "up", 0.1, collisions_bool= True, gripper= current_gripper, ignore_large_angle_checking= True):
                            rospy.loginfo("task completed successfully!")
                            control.pt1_instrument_pap_homing()
                            return True
                        else:
                            rospy.logwarn("error while planning z up...")
                            return False
                    else:
                        rospy.logwarn("error while planning y down...")
                        return False
            else:
                rospy.logwarn("error while planning z down...")
                return False
    else:
        rospy.logwarn("error while planning go to dragging coor")
        return False


def change_gripper(gripper_of_choice):
    global current_gripper
    if disable_gripper:
        if gripper_of_choice == PARALLEL:
            current_gripper = "parallel_gripper_manipulator"
        elif gripper_of_choice == SUCTION:
            current_gripper = "suction_gripper_manipulator"
        elif gripper_of_choice == DRAG:
            current_gripper = "drag_gripper_manipulator"
        
    else:
        if gripper_of_choice == PARALLEL:
            gripper_goal = GripperServerGoal()
            gripper_goal.task = gripper_goal.CHANGE_GRIPPER
            gripper_goal.option = gripper_goal.PARALLEL_GRIPPER
            gripper_client.send_goal(gripper_goal)
            gripper_client.wait_for_result()
            if gripper_client.get_result().success:
                current_gripper = "parallel_gripper_manipulator"
        elif gripper_of_choice == SUCTION:
            gripper_goal = GripperServerGoal()
            gripper_goal.task = gripper_goal.CHANGE_GRIPPER
            gripper_goal.option = gripper_goal.SUCTION_GRIPPER
            gripper_client.send_goal(gripper_goal)
            gripper_client.wait_for_result()
            if gripper_client.get_result().success:
                current_gripper = "suction_gripper_manipulator"
        elif gripper_of_choice == DRAG:
            gripper_goal = GripperServerGoal()
            gripper_goal.task = gripper_goal.CHANGE_GRIPPER
            gripper_goal.option = gripper_goal.DRAG_GRIPPER
            gripper_client.send_goal(gripper_goal)
            gripper_client.wait_for_result()
            if gripper_client.get_result().success:
                current_gripper = "drag_gripper_manipulator"
        else:
            print("invalid gripper")
            return False
        
def read_gripper_opening():
    gripper_goal = GripperServerGoal()
    gripper_goal.task = gripper_goal.GET_OPENING
    gripper_client.send_goal(gripper_goal)
    result = GripperServerResult()
    gripper_client.wait_for_result()
    result = gripper_client.get_result()
    if result.success:
        print("got opening result", result.opening)
        return result.opening
    else:
        return -1

def read_force_torque():
    return_list = []
    try:
        wrench = rospy.wait_for_message("/wrench", WrenchStamped, timeout=rospy.Duration(3.0))
        return_list = [wrench.wrench.force.x, wrench.wrench.force.y, wrench.wrench.force.z ]
    except rospy.exceptions.ROSException:
        rospy.logerr("timeout during wrench reading")
    return return_list

def spawn_collision(collision_type):
    if collision_type == "wetbay_1":
        control.add_stl("wetbay_1", [0.380205957278, -0.536709350446, 0.880732880111, -0.0340146575247, -0.00509989880644, -0.028132382271, 0.999012293805], "/home/piuslim373/all_ws/ur_ws/src/ur10_pap_description/meshes/210312_drawer.STL")
    elif collision_type == "wetbay_2":
        control.add_stl("wetbay_2", [0.405561080036, -0.186549459448, 0.894373577458, -0.0203836733802, -0.0296943239392, -0.0290327581976, 0.998929352826], "/home/piuslim373/all_ws/ur_ws/src/ur10_pap_description/meshes/210312_drawer.STL")
    elif collision_type == "wetbay_3":
        control.add_stl("wetbay_3", [0.417892085257, 0.162370185061, 0.872224417997, 0.00879190410812, -0.0155448046559, -0.031979654824, 0.999328956424], "/home/piuslim373/all_ws/ur_ws/src/ur10_pap_description/meshes/210312_drawer.STL")
    elif collision_type == "wetbay_4":
        control.add_stl("wetbay_4", [0.37936424045, -0.540887897195, 0.562035975711, -0.0338621564256, -0.000513314486854, -0.027517491513, 0.999047485624], "/home/piuslim373/all_ws/ur_ws/src/ur10_pap_description/meshes/210312_drawer.STL")
    elif collision_type == "wetbay_5":
        control.add_stl("wetbay_5", [0.399744827447, -0.190300916242, 0.569653866249, -0.00938372708013, -0.0413283972153, -0.0286316058997, 0.998691213736], "/home/piuslim373/all_ws/ur_ws/src/ur10_pap_description/meshes/210312_drawer.STL")
    elif collision_type == "wetbay_6":
        control.add_stl("wetbay_6", [0.41924960957, 0.158930219738, 0.555301717334, -0.00867069247397, -0.00785522250649, -0.0328477966377, 0.999391883511], "/home/piuslim373/all_ws/ur_ws/src/ur10_pap_description/meshes/210312_drawer.STL")
    return True

def list_to_pose(input_list):
    pose = Pose()
    pose.position.x = input_list[0]
    pose.position.y = input_list[1]
    pose.position.z = input_list[2]
    pose.orientation.x = input_list[3]
    pose.orientation.y = input_list[4]
    pose.orientation.z = input_list[5]
    pose.orientation.w = input_list[6]
    return pose

def z_down_touch():
    print("z down until limit switch pressed or when reached 1.5cm threshold")
    starting_z = control.get_coor(current_gripper, print_output = False).pose.position.z
    starting_wrench = rospy.wait_for_message("/wrench", WrenchStamped).wrench.force.z
    current_z = control.get_coor(current_gripper, print_output = False).pose.position.z
    current_wrench = rospy.wait_for_message("/wrench", WrenchStamped).wrench.force.z
    stop = False
    while (not stop) and (abs(current_z-starting_z) < 0.015):
        replan = 0
        for replan in range(3):
            if control.plan_cartesian_path("z", "down", 0.001, True, current_gripper):
                stop = rospy.wait_for_message("/ur_hardware_interface/io_states", IOStates).digital_in_states[3].state or (abs(starting_wrench - current_wrench) >= 3)
                current_z = control.get_coor(current_gripper, print_output = False).pose.position.z
                current_wrench = rospy.wait_for_message("/wrench", WrenchStamped).wrench.force.z
                continue
            else:
                if replan >= 2:
                    rospy.logerr("z down error")
                    return False
    if stop:
        rospy.loginfo("touched item")
    else:
        rospy.logwarn("reached 2cm threshold")
    return True
    
def kidney_dish_dumping(dumping_coor, motion_part):
    if control.plan_and_move_new(dumping_coor, collisions_bool= True, gripper= current_gripper):
        if motion_part == 1:
            weight_before = rospy.wait_for_message("/wrench", WrenchStamped).wrench.force.z
            angle = 120.0
            while angle >= 90:
                quaternion = tf.transformations.quaternion_from_euler(angle/180*pi, 0, 0)
                coor_to_go = [dumping_coor[0], dumping_coor[1], dumping_coor[2], quaternion[0], quaternion[1], quaternion[2], quaternion[3]]
                if control.plan_and_move_new(coor_to_go, collisions_bool= True, gripper= current_gripper):
                    weight_after = rospy.wait_for_message("/wrench", WrenchStamped).wrench.force.z
                    print(abs(weight_after - weight_before))
                    if abs(weight_after - weight_before) > 2:
                        print("dump successful")
                        break
                    weight_before = weight_after
                    angle -= 10
                else:
                    rospy.logwarn("cant reach target, pt1 might be at further end, retrying with motion part 3")
                    return "cant_reach"
            if control.plan_and_move_new(dumping_coor, collisions_bool= True, gripper= current_gripper):
                if control.pt1_homing():
                    return True
        elif motion_part == 2:
            quaternion = tf.transformations.quaternion_from_euler(90.0/180*pi, 0, 0)
            coor_to_go = [dumping_coor[0], dumping_coor[1], dumping_coor[2], quaternion[0], quaternion[1], quaternion[2], quaternion[3]]
            if control.plan_and_move_new(coor_to_go, collisions_bool= True, gripper= current_gripper):
                if control.plan_and_move_new(dumping_coor, collisions_bool= True, gripper= current_gripper):
                    if control.pt1_homing():
                        return True
        # elif motion_part == 3:          #in case where the processing table cant be reached
        #     weight_before = rospy.wait_for_message("/wrench", WrenchStamped).wrench.force.z
        #     angle = -30
        #     while angle >= -70:
        #         quaternion = tf.transformations.quaternion_from_euler(-pi, angle/180*pi, 0)
        #         coor_to_go = [dumping_coor[0], dumping_coor[1], dumping_coor[2], quaternion[0], quaternion[1], quaternion[2], quaternion[3]]
        #         if control.plan_and_move_new(coor_to_go, collisions_bool= True, gripper= current_gripper):
        #             weight_after = rospy.wait_for_message("/wrench", WrenchStamped).wrench.force.z
        #             print(abs(weight_after - weight_before))
        #             if abs(weight_after - weight_before) > 2:
        #                 print("dump successful")
        #                 break
        #             weight_before = weight_after
        #             angle -= 10
                # if control.plan_and_move_new(coor_to_go, collisions_bool= True, gripper= current_gripper):
                #     if control.plan_and_move_new(dumping_coor, collisions_bool= True, gripper= current_gripper):
                #         if control.pt1_homing():
                #             return True

def drybay_tray_placing(placing_coor):
    print("drybay tray placing in process")
    temp_coor = placing_coor
    temp_coor[1] += 0.03
    temp_coor[2] += 0.165
    if control.plan_and_move_new(placing_coor, collisions_bool= True, gripper= current_gripper) or control.plan_and_move_new(placing_coor, collisions_bool= True, gripper= current_gripper, ignore_large_angle_checking= True):
        if control.plan_cartesian_path_new("z", "down", 0.10, collisions_bool= True, gripper= current_gripper) or control.plan_cartesian_path_new("z", "down", 0.10, collisions_bool= True, gripper= current_gripper, ignore_large_angle_checking= True):
            if control.plan_cartesian_path_new("y", "down", 0.035, collisions_bool= True, gripper= current_gripper) or control.plan_cartesian_path_new("y", "down", 0.03, collisions_bool= True, gripper= current_gripper, ignore_large_angle_checking= True):
                if control.plan_cartesian_path_new("z", "down", 0.065, collisions_bool= True, gripper= current_gripper) or control.plan_cartesian_path_new("z", "down", 0.065, collisions_bool= True, gripper= current_gripper, ignore_large_angle_checking= True):                    
                    control.gripper_control("release")
                    if control.plan_cartesian_path_new("z", "up", 0.10, collisions_bool= True, gripper= current_gripper) or control.plan_cartesian_path_new("z", "up", 0.10, collisions_bool= True, gripper= current_gripper, ignore_large_angle_checking= True):
                        control.drybay_homing()
                        rospy.loginfo("tray placing successful")
                        return True
        else:
            rospy.logwarn("error during z down")
            return False
    else:
        rospy.logwarn("error during moving to destination")
        return False
             
################################################################################################## for debug

def debug_func(target_coor):
    if control.add_item("tray2", [0.1,0,0.02], [0.2, 0.3, 0.05]):
        print("item added")
        if control.attach_item("tray2", current_gripper):
            print("item attached")

    # weight_before = rospy.wait_for_message("/wrench", WrenchStamped).wrench.force.z
    # angle = 130.0
    # while angle >= 90:
    #     quaternion = tf.transformations.quaternion_from_euler(angle/180*pi, 0, 0)
    #     coor_to_go = [target_coor[0], target_coor[1], target_coor[2], quaternion[0], quaternion[1], quaternion[2], quaternion[3]]
    #     if control.plan_and_move_new(coor_to_go, collisions_bool= True, gripper= current_gripper):
    #         weight_after = rospy.wait_for_message("/wrench", WrenchStamped).wrench.force.z
    #         print(abs(weight_after - weight_before))
    #         if abs(weight_after - weight_before) > 2:
    #             print("dump successful")
    #             break
    #         weight_before = weight_after
    #         angle -= 10
    # control.pt1_instrument_pap_homing()
    return True
################################################################################################## for debug end



class MotionServer():
    def __init__(self):
        global gripper_client
        self.server = actionlib.SimpleActionServer('motion_server', RobotControlAction, self.execute, False)
        self.server.start()
        print("connecting to gripper action server...")
        gripper_client = actionlib.SimpleActionClient('gripper_server', GripperServerAction)
        if not gripper_client.wait_for_server(timeout = rospy.Duration(3.0)):
            print("gripper action not found and hence not connected")
        else:
            print("gripper action server connected!")

    def execute(self, goal):
        global current_gripper
        result = RobotControlResult()
        ########################################################################## motion server: HOME #####################33
        if goal.option == goal.HOME:
            result.success= control.homing()
        ########################################################################## motion server: placing item #####################33
        elif goal.option == goal.PLACE:
            try:
                extra_zdown = goal.extra_zdown
                pigeonhole_processing = goal.pigeonhole_processing
            except:
                extra_zdown = 0
                pigeonhole_processing = None
            try:
                placing_coor = [goal.target_pose.pose.position.x, goal.target_pose.pose.position.y, goal.target_pose.pose.position.z + 0.1, goal.target_pose.pose.orientation.x, goal.target_pose.pose.orientation.y, goal.target_pose.pose.orientation.z, goal.target_pose.pose.orientation.w]
                rospy.loginfo("placing item at x = " + str(placing_coor[0]) +", y = " + str(placing_coor[1]) +", z = "+ str(placing_coor[2]-0.1))
                rospy.loginfo("Gripper used: " + str(current_gripper))
                if placing(goal.starting_location, placing_coor, extra_zdown, pigeonhole_processing):
                    result.success= True
                else: 
                    result.success= False
            except:
                rospy.logerr("invalid input")
                result.success= False
        ########################################################################## motion server: special pick and place
        elif goal.option == goal.PLACE_SPECIAL:
            picking_coor = [None] * 7
            try:
                if goal.special_item == "kidney_dish":
                    motion_part = int(goal.extra_argument)
                    dumping_coor = [goal.target_pose.pose.position.x, goal.target_pose.pose.position.y, goal.target_pose.pose.position.z, goal.target_pose.pose.orientation.x, goal.target_pose.pose.orientation.y, goal.target_pose.pose.orientation.z, goal.target_pose.pose.orientation.w]
                    if kidney_dish_dumping(dumping_coor, motion_part):
                        result.success = True
                    else:
                        result.success = False
                elif goal.special_item == "drybay_tray":
                    placing_coor = [goal.target_pose.pose.position.x, goal.target_pose.pose.position.y, goal.target_pose.pose.position.z, goal.target_pose.pose.orientation.x, goal.target_pose.pose.orientation.y, goal.target_pose.pose.orientation.z, goal.target_pose.pose.orientation.w]
                    if drybay_tray_placing(placing_coor):
                        result.success= True
                    else: 
                        result.success= False
                elif goal.special_item == "pt1_fail_pick_release":
                    picking_coor = PoseStamped()
                    try:
                        picking_coor = [goal.target_pose.pose.position.x, goal.target_pose.pose.position.y, 1.1, goal.target_pose.pose.orientation.x, goal.target_pose.pose.orientation.y, goal.target_pose.pose.orientation.z, goal.target_pose.pose.orientation.w]
                    except:
                        print("coordinate for pt1 center not parsed in, using standard joint method instead")
                    if pt1_fail_pick_release(picking_coor):
                        result.success= True
                    else: 
                        result.success= False
                elif goal.special_item == "pt1_fail_pick_release_error":
                    picking_coor = PoseStamped()
                    if pt1_fail_pick_release(picking_coor, err = True):
                        result.success= True
                    else: 
                        result.success= False
                elif goal.special_item == "drybay_bracket":
                    motion_part = int(goal.extra_argument[-1])
                    which_bracket = int(goal.extra_argument[0])
                    bracket_coor = [goal.target_pose.pose.position.x, goal.target_pose.pose.position.y, goal.target_pose.pose.position.z, goal.target_pose.pose.orientation.x, goal.target_pose.pose.orientation.y, goal.target_pose.pose.orientation.z, goal.target_pose.pose.orientation.w]
                    tray_coor = [goal.reference_pose.pose.position.x, goal.reference_pose.pose.position.y, goal.reference_pose.pose.position.z, goal.reference_pose.pose.orientation.x, goal.reference_pose.pose.orientation.y, goal.reference_pose.pose.orientation.z, goal.reference_pose.pose.orientation.w]
                    pigeonhole_processing = goal.pigeonhole_processing
                    rospy.loginfo("placing scissor bracket at x = " + str(bracket_coor[0]) +", y = " + str(bracket_coor[1]) +", z = "+ str(bracket_coor[2]))
                    rospy.loginfo("Gripper used: " + str(current_gripper))
                    if drybay_scissor_brac_side_pushing(motion_part, which_bracket, bracket_coor, tray_coor, pigeonhole_processing):
                        result.success= True
                    else: 
                        result.success= False

                elif goal.special_item == "drybay_forcep_bracket":
                    motion_part = int(goal.extra_argument)
                    bracket_coor = [goal.target_pose.pose.position.x, goal.target_pose.pose.position.y, goal.target_pose.pose.position.z, goal.target_pose.pose.orientation.x, goal.target_pose.pose.orientation.y, goal.target_pose.pose.orientation.z, goal.target_pose.pose.orientation.w]
                    tray_coor = [goal.reference_pose.pose.position.x, goal.reference_pose.pose.position.y, goal.reference_pose.pose.position.z, goal.reference_pose.pose.orientation.x, goal.reference_pose.pose.orientation.y, goal.reference_pose.pose.orientation.z, goal.reference_pose.pose.orientation.w]
                    pigeonhole_processing = goal.pigeonhole_processing
                    rospy.loginfo("placing forcep bracket at x = " + str(bracket_coor[0]) +", y = " + str(bracket_coor[1]) +", z = "+ str(bracket_coor[2]))
                    rospy.loginfo("Gripper used: " + str(current_gripper))
                    if drybay_forcep_bracket_placing(motion_part, bracket_coor, tray_coor, pigeonhole_processing):
                        result.success= True
                    else: 
                        result.success= False
                elif goal.special_item == "drybay_mix_bracket":
                    bracket_coor = [goal.target_pose.pose.position.x, goal.target_pose.pose.position.y, goal.target_pose.pose.position.z, goal.target_pose.pose.orientation.x, goal.target_pose.pose.orientation.y, goal.target_pose.pose.orientation.z, goal.target_pose.pose.orientation.w]
                    tray_coor = [goal.reference_pose.pose.position.x, goal.reference_pose.pose.position.y, goal.reference_pose.pose.position.z, goal.reference_pose.pose.orientation.x, goal.reference_pose.pose.orientation.y, goal.reference_pose.pose.orientation.z, goal.reference_pose.pose.orientation.w]
                    pigeonhole_processing = goal.pigeonhole_processing
                    rospy.loginfo("placing mix bracket at x = " + str(bracket_coor[0]) +", y = " + str(bracket_coor[1]) +", z = "+ str(bracket_coor[2]))
                    rospy.loginfo("Gripper used: " + str(current_gripper))
                    if drybay_mix_bracket_placing(bracket_coor, tray_coor, pigeonhole_processing):
                        result.success= True
                    else: 
                        result.success= False
            except:
                rospy.logerr("invalid input")
                result.success= False
        ########################################################################## motion server: picking item #####################33
        elif goal.option == goal.PICK:
            picking_coor = [None] * 7
            try:
                extra_zdown = goal.extra_zdown
                pigeonhole_processing = goal.pigeonhole_processing
            except:
                extra_zdown = 0
                pigeonhole_processing = None
            try:
                picking_coor = [goal.target_pose.pose.position.x, goal.target_pose.pose.position.y, goal.target_pose.pose.position.z + 0.1, goal.target_pose.pose.orientation.x, goal.target_pose.pose.orientation.y, goal.target_pose.pose.orientation.z, goal.target_pose.pose.orientation.w]
                rospy.loginfo("picking item at x = " + str(picking_coor[0]) +", y = " + str(picking_coor[1]) +", z = "+ str(picking_coor[2]-0.1))
                rospy.loginfo("Gripper used: " + str(current_gripper))
                if picking(goal.starting_location, picking_coor, extra_zdown, pigeonhole_processing):
                    result.success= True
                else: 
                    result.success= False
            except:
                rospy.logerr("invalid input")
                result.success= False
        ########################################################################## motion server: special pick and place
        elif goal.option == goal.PICK_SPECIAL:
            picking_coor = [None] * 7
            try:
                extra_zdown = goal.extra_zdown
                pigeonhole_processing = goal.pigeonhole_processing
                item = goal.extra_argument
                starting_location = goal.starting_location
            except:
                extra_zdown = 0
                pigeonhole_processing = None
                item = ""
                starting_location = ""
            try:
                if goal.special_item == "wetbay_picking":
                    picking_coor = [goal.target_pose.pose.position.x, goal.target_pose.pose.position.y, goal.target_pose.pose.position.z + 0.1, goal.target_pose.pose.orientation.x, goal.target_pose.pose.orientation.y, goal.target_pose.pose.orientation.z, goal.target_pose.pose.orientation.w]
                    rospy.loginfo("picking wetbay item at x = " + str(picking_coor[0]) +", y = " + str(picking_coor[1]) +", z = "+ str(picking_coor[2]-0.1))
                    rospy.loginfo("Gripper used: " + str(current_gripper))
                    if wetbay_picking(picking_coor, extra_zdown, item, pigeonhole_processing):
                        result.success= True
                    else: 
                        result.success= False
                
                if goal.special_item == "drybay_picking":
                    picking_coor = [goal.target_pose.pose.position.x, goal.target_pose.pose.position.y, goal.target_pose.pose.position.z + 0.1, goal.target_pose.pose.orientation.x, goal.target_pose.pose.orientation.y, goal.target_pose.pose.orientation.z, goal.target_pose.pose.orientation.w]
                    rospy.loginfo("picking drybay item at x = " + str(picking_coor[0]) +", y = " + str(picking_coor[1]) +", z = "+ str(picking_coor[2]-0.1))
                    rospy.loginfo("Gripper used: " + str(current_gripper))
                    if drybay_picking(picking_coor, extra_zdown, item, starting_location):
                        result.success= True
                    else: 
                        result.success= False
            except:
                rospy.logerr("invalid input")
                result.success= False
        ########################################################################## motion server: drag #####################33
        elif goal.option == goal.DRAG_TASK:
            instrument_coor = [None] * 7
            try:
                instrument_coor = [goal.target_pose.pose.position.x, goal.target_pose.pose.position.y, goal.target_pose.pose.position.z+0.06, goal.target_pose.pose.orientation.x, goal.target_pose.pose.orientation.y, goal.target_pose.pose.orientation.z, goal.target_pose.pose.orientation.w]
                drag_direction = goal.special_item
                rospy.loginfo("dragging instrument at x = " + str(instrument_coor[0]) +", y = " + str(instrument_coor[1]) +", z = "+ str(instrument_coor[2]-0.06))
                if drag_instrument(instrument_coor, drag_direction):
                    result.success= True
                else: 
                    result.success= False
            except:
                rospy.logerr("invalid input")
                result.success= False
        ########################################################################## motion server: touchpoint #####################33
        elif goal.option == goal.TOUCH_POINT:
            touching_coor = [None] * 7
            try:
                touching_coor = [goal.target_pose.pose.position.x, goal.target_pose.pose.position.y, goal.target_pose.pose.position.z+0.1, goal.target_pose.pose.orientation.x, goal.target_pose.pose.orientation.y, goal.target_pose.pose.orientation.z, goal.target_pose.pose.orientation.w]
                # current_gripper = "parallel_gripper_manipulator"
                rospy.loginfo("touching point at x = " + str(touching_coor[0]) +", y = " + str(touching_coor[1]) +", z = "+ str(touching_coor[2]-0.2))
                if touching_point(touching_coor):
                    result.success= True
                else: 
                    result.success= False
            except:
                rospy.logerr("invalid input")
                result.success= False
        ########################################################################## motion server: get status #####################33
        elif goal.option == goal.STATUS:
            control.get_coor()
            control.get_joint()
            result.success = True
        ########################################################################## motion server: debug function #####################33
        elif goal.option == goal.DEBUG:
            print("debug function running")
            target_coor = [goal.target_pose.pose.position.x, goal.target_pose.pose.position.y, goal.target_pose.pose.position.z, goal.target_pose.pose.orientation.x, goal.target_pose.pose.orientation.y, goal.target_pose.pose.orientation.z, goal.target_pose.pose.orientation.w]
            print(target_coor)
            debug_func(target_coor)
            result.success = True
        ########################################################################## motion server: changing gripper #####################33
        elif goal.option == goal.CHANGE_GRIPPER:
            try:
                change_gripper(goal.gripper)
                result.success = True   
            except:
                print("something is wrong, try again")
                result.success = False 
        ########################################################################## motion server: go to coor #####################33
        elif goal.option == goal.GO_TO_LOCATION:
            target_pose = [goal.target_pose.pose.position.x, goal.target_pose.pose.position.y, goal.target_pose.pose.position.z, goal.target_pose.pose.orientation.x, goal.target_pose.pose.orientation.y, goal.target_pose.pose.orientation.z, goal.target_pose.pose.orientation.w]
            print(target_pose)
            if control.plan_and_move_new(target_pose,collisions_bool= True,gripper= current_gripper) or control.plan_and_move_new(target_pose,collisions_bool= True,gripper= current_gripper, ignore_large_angle_checking= True):
                result.success = True
            else:
                result.success = False
        ########################################################################## motion server: go to wetbay #####################33
        elif goal.option == goal.GO_TO_WETBAY:
            result.success= control.wetbay_homing()
        ########################################################################## motion server: drybay #####################33
        elif goal.option == goal.GO_TO_DRYBAY:
            result.success= control.drybay_homing()
        ########################################################################## motion server: processing table1 #####################33
        elif goal.option == goal.GO_TO_PT1:
            result.success= control.pt1_homing()
        ########################################################################## motion server: processing table1 #####################33
        elif goal.option == goal.GO_TO_PT1_INSTRUMENT_PAP:
            result.success= control.pt1_instrument_pap_homing()
        ########################################################################## motion server: processing table 2#####################33
        elif goal.option == goal.GO_TO_PT2:
            result.success= control.pt2_homing()
        ########################################################################## motion server: read gripper opening #####################33
        elif goal.option == goal.READ_GRIPPER_OPENING:
            result.gripper_opening = read_gripper_opening()
            if (result.gripper_opening != -1):
                result.success= True
            else: 
                result.success= False
        ########################################################################## motion server: force torque #####################33
        elif goal.option == goal.READ_FORCE_TORQUE:
            result.force_torque = read_force_torque()
            rospy.loginfo(result.force_torque)
            if (result.force_torque != []):
                result.success= True
            else: 
                result.success= False
        ########################################################################## motion server: spawn collision #####################33
        elif goal.option == goal.SPAWN_COLLISION:
            if spawn_collision(goal.collision):
                result.success= True
            else: 
                result.success= False
        ########################################################################## motion server: spawn collision #####################33
        elif goal.option == goal.DASHBOARD_SERVER_QUERY:
            dashboard_return = dashboard.dashboard_task("general_query")
            if len(dashboard_return) == 2:
                result.dashboard_robot_status = dashboard_return[0]
                result.dashboard_program_status = dashboard_return[1]
                result.success= True
            else: 
                result.success= False
        ########################################################################## motion server: spawn collision #####################33
        elif goal.option == goal.DASHBOARD_SERVER_ACTION:
            task = goal.extra_argument
            if dashboard.dashboard_command(task):
                result.success= True
            else: 
                result.success= False
        else:
            print("invalid option")
            result.success= False
        
        

        self.server.set_succeeded(result)

class DashboardServer():
    def socket_command(self, command):
        '''socket command is older version of communication with UR, some command like set speed is not available from dashboard_server is sent with this'''
        sock =socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect((UR_DASHBOARD_SERVER_IP, UR_SOCKET_SERVER_PORT))
        if command == "set_speed":
            sock.send(("set speed 0.75"+ '\n').encode('utf-8'))
            print("speed adjusted to 75%")
            return True

    def dashboard_command(self, command):
        '''Dashboard command, not all command is available here, hence will need conventional socket command (diff port)'''
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_address = (UR_DASHBOARD_SERVER_IP, UR_DASHBOARD_SERVER_PORT)
        sock.connect(server_address)
        receive = sock.recv(2048)
        print(receive)
        sock.send(command+ "\n")
        sock.settimeout(7)
        try:
            receive = sock.recv(2048)
            print(receive)
            if(command == "power on" or command == "load robotcontroller.urp" or command == "brake release"):
                return True
            elif(command == "robotmode" ):
                return receive[11:].strip('\n')
            elif (command == "programstate"):
                if "PLAYING" in receive:
                    return "PLAYING"
                elif "PAUSED" in receive:
                    return "PAUSE"
                elif "STOPPED" in receive:
                    return "STOPPED"
                else:
                    rospy.logerr("unknown program state", receive)
                    return receive
            elif(command == "pause"):
                if "Pausing" in receive:
                    return True
                else: 
                    return False
            elif(command == "play"):
                if "Starting" in receive:
                    return True
                else: 
                    return False
                
        except socket.timeout:
            print("caught a time out")
            return False

    def dashboard_task(self, action, command=""):
        if action == "init":
            ur10_socket = None
            connected = False
            for i in range(10):
                try:
                    ur10_socket = socket.socket()
                    ur10_socket.connect((UR_DASHBOARD_SERVER_IP, UR_DASHBOARD_SERVER_PORT))
                    print("connected")
                    connected = True
                    break
                except:
                    print("connection failed, retrying")
                    sleep(0.5)
                    i += 1
            if not connected:
                rospy.logerr("couldnt establish connection to ur10, is it power on?")
                return False
            else:
                if self.dashboard_command("power on") and self.dashboard_command("load robotcontroller.urp") and self.dashboard_command("brake release") and self.socket_command("set_speed"):
                    print("waiting 15sec for brake release")
                    sleep(15)
                    if self.dashboard_command("play"):
                        return True
                else:
                    return False
        if action == "general_query":
            return [self.dashboard_command("robotmode"), self.dashboard_command("programstate")]

def shutdown_robot():
    print("powering off robot")
    dashboard.dashboard_command("power off")

if __name__ == "__main__":
    
    # print("waiting for gripper server")
    # rospy.wait_for_service('/comm_gripper/gripper_server')
    # print("server connected")
    
    # control.homing()
    # control.add_stl("wetbay_1", [0.380205957278, -0.536709350446, 0.880732880111, -0.0340146575247, -0.00509989880644, -0.028132382271, 0.999012293805], "/home/piuslim373/all_ws/ur_ws/src/ur10_pap_description/meshes/210312_drawer.STL")
    # control.add_stl("wetbay_2", [0.405561080036, -0.186549459448, 0.894373577458, -0.0203836733802, -0.0296943239392, -0.0290327581976, 0.998929352826], "/home/piuslim373/all_ws/ur_ws/src/ur10_pap_description/meshes/210312_drawer.STL")
    # control.add_stl("wetbay_3", [0.417892085257, 0.162370185061, 0.872224417997, 0.00879190410812, -0.0155448046559, -0.031979654824, 0.999328956424], "/home/piuslim373/all_ws/ur_ws/src/ur10_pap_description/meshes/210312_drawer.STL")
    # control.add_stl("wetbay_4", [0.37936424045, -0.540887897195, 0.562035975711, -0.0338621564256, -0.000513314486854, -0.027517491513, 0.999047485624], "/home/piuslim373/all_ws/ur_ws/src/ur10_pap_description/meshes/210312_drawer.STL")
    # control.add_stl("wetbay_5", [0.399744827447, -0.190300916242, 0.569653866249, -0.00938372708013, -0.0413283972153, -0.0286316058997, 0.998691213736], "/home/piuslim373/all_ws/ur_ws/src/ur10_pap_description/meshes/210312_drawer.STL")
    # control.add_stl("wetbay_6", [0.41924960957, 0.158930219738, 0.555301717334, -0.00867069247397, -0.00785522250649, -0.0328477966377, 0.999391883511], "/home/piuslim373/all_ws/ur_ws/src/ur10_pap_description/meshes/210312_drawer.STL")
    rospy.init_node('ur_motion_server', anonymous=True)
    server = MotionServer()
    control = MoveGroupPythonInteface()
    
    rospy.loginfo("Motion Server ready")