#!/usr/bin/env python

# action server here
# from builtins import print, range
from ast import literal_eval
from yaml import dump
import rospy
from ur5_pap.msg import *
from ur5_pap.srv import *
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

PARALLEL = 50
SUCTION = 51
DRAG = 52

picking_joint = None
current_gripper = "sb_gripper_manipulator"
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

        sb_gripper_manipulator = moveit_commander.MoveGroupCommander(
            "sb_gripper_manipulator")
        locker_gripper_manipulator = moveit_commander.MoveGroupCommander(
            "locker_gripper_manipulator")

        display_trajectory_publisher = rospy.Publisher(
            '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)

        self.box_name = ''
        self.robot = robot
        self.scene = scene

        self.sb_gripper_manipulator = sb_gripper_manipulator
        self.locker_gripper_manipulator = locker_gripper_manipulator

    def display_marker(self, marker_position, marker_orien, step):
        global markers
        i = 0
        k = 0
        for i in range(step):
            pose = Pose(Point(marker_position[k], marker_position[k+1], marker_position[k+2]), Quaternion(
                marker_orien[0], marker_orien[1], marker_orien[2], marker_orien[3]))
            axis_length = 0.1
            axis_radius = 0.01
            markers.publishAxis(pose, axis_length, axis_radius, 5.0)
            k += 3
        return 1

    def cleanup_node(self):
        global markers
        markers.deleteAllMarkers()
        return 1

################################################################################## Gripper dependant ###########

    def gripper_control(self, action):
        if disable_gripper:
            return True
        gripper_goal = GripperServerGoal()
        if action == "grip":
            gripper_goal.task = gripper_goal.GRIPPER_ACTION
            gripper_goal.motion_task = gripper_goal.CLOSE_GRIPPER
        elif action == "release":
            gripper_goal.task = gripper_goal.GRIPPER_ACTION
            gripper_goal.motion_task = gripper_goal.OPEN_GRIPPER
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
                rospy.logwarn(
                    "Gripper server timeout, signal to re-open gripper server")
                # Code to reopen gripper server
                #
                #
                #
                return False
        except:
            return False

    def homing(self):
        group = eval("self." + current_gripper)
        joint_goal = group.set_named_target("home")
        go_result = group.go(joint_goal, wait=True)
        group.stop()
        return go_result

    def sparebay_homing(self):
        group = eval("self." + current_gripper)
        joint_goal = group.set_named_target("sparebay_home")
        go_result = group.go(joint_goal, wait=True)
        group.stop()
        return go_result

    def move_to_saved_joint(self, saved_name):
        group = eval("self." + current_gripper)
        joint_goal = group.set_named_target(saved_name)
        go_result = group.go(joint_goal, wait=True)
        group.stop()
        return go_result

    def move_with_joint(self, joint, angle, manipulator="sb_gripper_manipulator"):
        group = eval("self." + manipulator)
        joint_goal = group.get_current_joint_values()
        joint_goal[joint] = angle
        go_result = group.go(joint_goal, wait=True)
        group.stop()
        return go_result

    def move_all_joint(self, angle, manipulator="sb_gripper_manipulator"):
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
        print(go_result)
        return go_result

    def plan_cartesian_path(self, axis, direction, distance, collisions_bool=True, gripper="sb_gripper_manipulator", ignore_large_angle_checking=False):
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
            (path, fraction) = group.compute_cartesian_path(
                waypoints, 0.01, 0, avoid_collisions=collisions_bool)

            if ignore_large_angle_checking and fraction >= 0.95:
                large_turning = False
                num_point = len(path.joint_trajectory.points)
                for i in range(num_point):
                    if i > 0:
                        if path.joint_trajectory.points[i].positions[1] >= -0.2:
                            large_turning = True
                            print("new plan cartesian debug", str(
                                path.joint_trajectory.points[i].positions[1]), str(i))
                            rospy.logwarn(
                                "Large turning is predicted, retrying")
                            break
                if not large_turning:
                    rospy.logwarn("ignoring large angle check engaged!")
                    speedup_path = self.speedup(path)
                    group.execute(speedup_path, wait=True)
                    return True

            if fraction >= 0.9:
                print("fraction = " + str(fraction))
                num_point = len(path.joint_trajectory.points)
                large_turning = False
                for i in range(num_point):
                    if i > 0:
                        check_joint_0 = abs(
                            path.joint_trajectory.points[i].positions[0] - path.joint_trajectory.points[0].positions[0])
                        check_joint_1 = path.joint_trajectory.points[i].positions[1]
                        check_joint_2 = path.joint_trajectory.points[i].positions[2]
                        check_joint_3 = abs(
                            path.joint_trajectory.points[i].positions[3] - path.joint_trajectory.points[0].positions[3])
                        if ((check_joint_0 > 1.745) or (check_joint_1 >= -0.2) or (check_joint_2 <= 0) or (check_joint_3 > 0.3)):
                            large_turning = True
                            rospy.logwarn(
                                "Large turning is predicted, retrying")
                            if (check_joint_0 > 1.745):
                                rospy.logerr(
                                    "check_joint_0 large turning error caught" + str(check_joint_0))
                            if (check_joint_1 >= -0.2):
                                rospy.logerr(
                                    "check_joint_1 large turning error caught" + str(check_joint_1))
                            if (check_joint_2 <= 0):
                                rospy.logerr(
                                    "check_joint_2 large turning error caught" + str(check_joint_2))
                            if (check_joint_3 > 0.3):
                                rospy.logerr(
                                    "check_joint_3 large turning error caught" + str(check_joint_3))
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

    def check_plan_without_move(self, end_pose, collisions_bool=True, gripper="sb_gripper_manipulator", ignore_large_angle_checking=False):
        for replan in range(3):
            global stop_marker
            group = eval("self." + gripper)
            joint = group.get_current_pose()

            start_position = [joint.pose.position.x,
                              joint.pose.position.y, joint.pose.position.z]
            start_orien = [joint.pose.orientation.x, joint.pose.orientation.y,
                           joint.pose.orientation.z, joint.pose.orientation.w]
            end_position = [end_pose[0], end_pose[1], end_pose[2]]
            end_orien = [end_pose[3], end_pose[4], end_pose[5], end_pose[6]]

            marker_position, step = pose_generator.main(
                start_position, end_position)
            stop_marker = False
            t = threading.Thread(target=self.display_marker, args=(
                marker_position, end_orien, step,))
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

            path, fraction = group.compute_cartesian_path(
                pose_list, 0.01, 0.0, avoid_collisions=collisions_bool)
            if fraction == 1:
                return True

        return False

    def plan_and_move(self, end_pose, collisions_bool=True, gripper="sb_gripper_manipulator", ignore_large_angle_checking=False):
        for replan in range(3):
            global stop_marker
            group = eval("self." + gripper)
            joint = group.get_current_pose()

            start_position = [joint.pose.position.x,
                              joint.pose.position.y, joint.pose.position.z]
            start_orien = [joint.pose.orientation.x, joint.pose.orientation.y,
                           joint.pose.orientation.z, joint.pose.orientation.w]
            end_position = [end_pose[0], end_pose[1], end_pose[2]]
            end_orien = [end_pose[3], end_pose[4], end_pose[5], end_pose[6]]

            marker_position, step = pose_generator.main(
                start_position, end_position)
            stop_marker = False
            t = threading.Thread(target=self.display_marker, args=(
                marker_position, end_orien, step,))
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

            path, fraction = group.compute_cartesian_path(
                pose_list, 0.01, 0.0, avoid_collisions=collisions_bool)

            if ignore_large_angle_checking and fraction >= 0.95:
                large_turning = False
                num_point = len(path.joint_trajectory.points)
                for i in range(num_point):
                    if i > 0:
                        if path.joint_trajectory.points[i].positions[1] >= -0.2:
                            large_turning = True
                            print("new plan cartesian debug", str(
                                path.joint_trajectory.points[i].positions[1]), str(i))
                            rospy.logwarn(
                                "Large turning is predicted, retrying")
                            break
                if not large_turning:
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
                        check_joint_0 = abs(
                            path.joint_trajectory.points[i].positions[0] - path.joint_trajectory.points[0].positions[0])
                        check_joint_1 = path.joint_trajectory.points[i].positions[1]
                        check_joint_2 = path.joint_trajectory.points[i].positions[2]
                        check_joint_3 = abs(
                            path.joint_trajectory.points[i].positions[3] - path.joint_trajectory.points[0].positions[3])
                        if ((check_joint_0 > 1.745) or (check_joint_1 >= -0.2) or (check_joint_2 <= 0) or (check_joint_3 > 2)):
                            large_turning = True
                            rospy.logwarn(
                                "Large turning is predicted, retrying")
                            if (check_joint_0 > 1.745):
                                rospy.logerr(
                                    "check_joint_0 large turning error caught" + str(check_joint_0))
                            if (check_joint_1 >= -0.2):
                                rospy.logerr(
                                    "check_joint_1 large turning error caught" + str(check_joint_1))
                            if (check_joint_2 <= 0):
                                rospy.logerr(
                                    "check_joint_2 large turning error caught" + str(check_joint_2))
                            if (check_joint_3 > 0.3):
                                rospy.logerr(
                                    "check_joint_3 large turning error caught" + str(check_joint_3))
                            break

                if not large_turning:
                    speedup_path = self.speedup(path)
                    execute_result = group.execute(speedup_path, wait=True)
                    stop_marker = True
                    t.join()
                    self.cleanup_node()
                    return execute_result
        return False

    def rotate_joint(self, target_quatenion, manipulator="sb_gripper_manipulator"):
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

    def get_coor(self, manipulator="sb_gripper_manipulator", print_output=True):
        group = eval("self." + current_gripper)
        joint = group.get_current_pose()
        if print_output:
            print("=========== pose")
            print(joint)
            print(joint.pose.position.x, joint.pose.position.y, joint.pose.position.z, joint.pose.orientation.x,
                  joint.pose.orientation.y, joint.pose.orientation.z, joint.pose.orientation.w)
        return joint

    def get_joint(self, manipulator="sb_gripper_manipulator"):
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

# pre plan motion


def change_gripper(gripper_of_choice):
    global current_gripper
    if disable_gripper:
        if gripper_of_choice == SB_GRIPPER:
            current_gripper = "sb_gripper_manipulator"
        elif gripper_of_choice == LOCKER_GRIPPER:
            current_gripper = "locker_gripper_manipulator"
    else:
        if gripper_of_choice == SB_GRIPPER:
            gripper_goal = GripperServerGoal()
            gripper_goal.task = gripper_goal.CHANGE_GRIPPER
            gripper_goal.motion_task = gripper_goal.SB_GRIPPER
            gripper_client.send_goal(gripper_goal)
            gripper_client.wait_for_result()
            if gripper_client.get_result().success:
                current_gripper = "sb_gripper_manipulator"
        elif gripper_of_choice == SUCTION:
            gripper_goal = GripperServerGoal()
            gripper_goal.task = gripper_goal.CHANGE_GRIPPER
            gripper_goal.motion_task = gripper_goal.LOCKER_GRIPPER
            gripper_client.send_goal(gripper_goal)
            gripper_client.wait_for_result()
            if gripper_client.get_result().success:
                current_gripper = "locker_gripper_manipulator"

        else:
            print("invalid gripper")
            return False


def read_force_torque():
    return_list = []
    try:
        wrench = rospy.wait_for_message(
            "/wrench", WrenchStamped, timeout=rospy.Duration(3.0))
        return_list = [wrench.wrench.force.x,
                       wrench.wrench.force.y, wrench.wrench.force.z]
    except rospy.exceptions.ROSException:
        rospy.logerr("timeout during wrench reading")
    return return_list


# for debug

def debug_func(target_coor):
    if control.add_item("tray2", [0.1, 0, 0.02], [0.2, 0.3, 0.05]):
        print("item added")
        if control.attach_item("tray2", current_gripper):
            print("item attached")

    # weight_before = rospy.wait_for_message("/wrench", WrenchStamped).wrench.force.z
    # angle = 130.0
    # while angle >= 90:
    #     quaternion = tf.transformations.quaternion_from_euler(angle/180*pi, 0, 0)
    #     coor_to_go = [target_coor[0], target_coor[1], target_coor[2], quaternion[0], quaternion[1], quaternion[2], quaternion[3]]
    #     if control.plan_and_move(coor_to_go, collisions_bool= True, gripper= current_gripper):
    #         weight_after = rospy.wait_for_message("/wrench", WrenchStamped).wrench.force.z
    #         print(abs(weight_after - weight_before))
    #         if abs(weight_after - weight_before) > 2:
    #             print("dump successful")
    #             break
    #         weight_before = weight_after
    #         angle -= 10
    # control.pt1_instrument_pap_homing()
    return True
# for debug end


class MotionServer():
    def __init__(self):
        global gripper_client
        # self.server = actionlib.SimpleActionServer('motion_server', RobotControlAction, self.execute, False)
        # self.server.start()
        rospy.Service('motion_server', MotionService, self.execute)
        print("connecting to gripper action server...")
        # gripper_client = actionlib.SimpleActionClient('gripper_server', GripperServerAction)
        # if not gripper_client.wait_for_server(timeout = rospy.Duration(3.0)):
        #     print("gripper action not found and hence not connected")
        # else:
        #     print("gripper action server connected!")

    def execute(self, goal):
        global current_gripper
        print("server called")
        result = MotionServiceResponse()
        # motion server: homing #####################33
        if goal.motion_task == "saved_joint":
            if goal.payload == "home":
                result.success = control.homing()
            elif goal.payload == "sparebay_home":
                result.success = control.sparebay_homing()
        # motion server: sparebay homing #####################33
        elif goal.motion_task == "saved_cartesian":
            # put some dummy data here
            dispose_box = [-0.3, 0.1, 0.5, 0.707, 0.707, 0, 0]
            result.success = control.plan_and_move(
                eval(goal.payload), ignore_large_angle_checking=True)
        # motion server: sparebay homing #####################33
        elif goal.motion_task == "custom_joint":
            joints = literal_eval(goal.payload)
            result.success = control.move_all_joint(joints)
        # motion server: sparebay homing #####################33
        elif goal.motion_task == "custom_cartesian":
            pose = literal_eval(goal.payload)
            result.success = control.plan_and_move(
                pose, ignore_large_angle_checking=True)
        # motion server: sparebay homing #####################33
        elif goal.motion_task == "jog":
            print("not ready")
        # motion server: get status #####################33
        elif goal.motion_task == "status":
            current_pose = control.get_coor()

            result.current_pose = [current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z,
                                   current_pose.pose.orientation.x, current_pose.pose.orientation.y, current_pose.pose.orientation.z, current_pose.pose.orientation.w]
            result.current_joint = control.get_joint()
            print(control.get_joint())
            # result.force_torque = read_force_torque()
            result.success = True
        # motion server: changing gripper #####################33
        elif goal.motion_task == "change_gripper":
            try:
                change_gripper(goal.gripper)
                result.success = True
            except:
                print("something is wrong, try again")
                result.success = False
        # motion server: debug function #####################33
        elif goal.motion_task == "debug":
            print("debug function running")
            result.success = True

        # motion server: spawn collision #####################33
        elif goal.motion_task == "dashboard_query":
            dashboard_return = dashboard.dashboard_task("general_query")
            if len(dashboard_return) == 3:
                result.dashboard_robot_status = dashboard_return[0]
                result.dashboard_program_status = dashboard_return[1]
                result.dashboard_safety_status = dashboard_return[2]
                result.success = True
            else:
                result.success = False
        # motion server: spawn collision #####################33
        elif goal.motion_task == "dashboard_action":
            pass
            # task = goal.extra_argument
            # # if dashboard.dashboard_command(task): #uodated to dashboard_task, to check if some other program is suing the old dashboard command
            # if dashboard.dashboard_task(task):
            #     result.success= True
            # else:
            #     result.success= False
        else:
            print("invalid option")
            result.success = False

        return result


class DashboardServer():
    def socket_command(self, command):
        '''socket command is older version of communication with UR, some command like set speed is not available from dashboard_server is sent with this'''
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect((UR_DASHBOARD_SERVER_IP, UR_SOCKET_SERVER_PORT))
        if command == "set_speed":
            sock.send(("set speed 0.75" + '\n').encode('utf-8'))
            print("speed adjusted to 75%")
            return True

    def dashboard_command(self, command):
        '''Dashboard command, not all command is available here, hence will need conventional socket command (diff port)'''
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_address = (UR_DASHBOARD_SERVER_IP, UR_DASHBOARD_SERVER_PORT)
        sock.connect(server_address)
        receive = sock.recv(2048)
        print(receive)
        sock.send(command + "\n")
        sock.settimeout(7)
        try:
            receive = sock.recv(2048)
            print(receive)
            if(command == "power on" or command == "load robotcontroller.urp" or command == "brake release"):
                return True
            elif(command == "robotmode"):
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
            elif (command == "safetystatus"):
                return receive[14:].strip('\n')
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
            elif command == "unlock protective stop":
                if "releasing" in receive:
                    return True
                else:
                    return False
            elif command == "close popup" or command == "close safety popup":
                if "closing" in receive:
                    return True
                else:
                    return False

        except socket.timeout:
            print("caught a time out")
            return False

    def dashboard_task(self, action):
        if action == "init":
            ur10_socket = None
            connected = False
            for i in range(10):
                try:
                    ur10_socket = socket.socket()
                    ur10_socket.connect(
                        (UR_DASHBOARD_SERVER_IP, UR_DASHBOARD_SERVER_PORT))
                    print("connected")
                    connected = True
                    break
                except:
                    print("connection failed, retrying")
                    sleep(0.5)
                    i += 1
            if not connected:
                rospy.logerr(
                    "couldnt establish connection to ur10, is it power on?")
                return False
            else:
                if self.dashboard_command("power on") and self.dashboard_command("load robotcontroller.urp") and self.dashboard_command("brake release") and self.socket_command("set_speed"):
                    print("waiting 15sec for brake release")
                    sleep(15)
                    if self.dashboard_command("play"):
                        return True
                else:
                    return False
        elif action == "general_query":
            return [self.dashboard_command("robotmode"), self.dashboard_command("programstate"), self.dashboard_command("safetystatus")]
        elif action == "release_protective_or_estop":
            # check if robot is estop or protective stop, if former need release brake before play
            # with estop depressed, normal if wihtout estop depressed
            if self.dashboard_command("safetystatus") == "ROBOT_EMERGENCY_STOP":
                rospy.logerr(
                    "ESTOP need to be unpressed before activating robot")
                return False
            elif self.dashboard_command("safetystatus") == "PROTECTIVE_STOP":
                rospy.loginfo(
                    "Robot reached a protective stop, waiting for 5 sec before releasing")
                sleep(5)
                return self.dashboard_command("close safety popup") and self.dashboard_command("unlock protective stop") and self.dashboard_command("play")
            elif self.dashboard_command("safetystatus") == "NORMAL":
                if self.dashboard_command("close safety popup") and self.dashboard_command("brake release"):
                    sleep(5)
                    if self.dashboard_command("play"):
                        return True
                return False


def shutdown_robot():
    print("powering off robot")
    dashboard.dashboard_command("power off")
    sleep(1)
    # dashboard.dashboard_command("shutdown")


if __name__ == "__main__":

    rospy.init_node('ur_motion_server')
    # server = None
    # control = None
    # dashboard = DashboardServer()
    # if dashboard.dashboard_task("init"):
    #     server = MotionServer()
    #     control = MoveGroupPythonInteface()
    #     rospy.loginfo("Motion Server ready")
    # rospy.on_shutdown(shutdown_robot)

    server = MotionServer()
    # rospy.Service('motion_service', MotionService, service_cb)
    control = MoveGroupPythonInteface()
    rospy.loginfo("Motion Server ready")
    rospy.spin()
