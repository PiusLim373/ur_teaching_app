#!/usr/bin/env python
from ast import literal_eval
from sre_constants import SUCCESS
import rospy
from ur5_pap.srv import *
from tree_executor.srv import *
import rospkg
import json
from threading import Thread
import requests
from collections import OrderedDict
import yaml
import random

NODEJS_MISSION_CONTROL_URL = "http://localhost:5000/api/mission_control/detail/"


class TreeExeClass():
    def __init__(self):
        self.motion_client = rospy.ServiceProxy('/motion_server', MotionService)

        self.control_node_type = ['sequence', 'fallback']
        self.motion_testing_success = False
        self.child_dict = {}
        self.sorted_child_dict = {}
        self.tree = None
        self.id = ""
        self.tasks = []

        rospy.Service('/tree_executor', TreeExecutorService, self.execute_cb)

# ================================================================= UTILS =============

    def search_dict_by_id(self, id):
        for index, x in enumerate(self.tasks):
            if x['id'] == id:
                return index

    def build_tree(self, id):
        temp_node = self.tasks[self.search_dict_by_id(id)]
        # print(temp_node)
        if not id in self.sorted_child_dict.keys():
            return temp_node
        for child_id in self.sorted_child_dict[id]:
            # print(child_id)
            temp_node['data']['ros_payload']['children'].append(
                self.build_tree(child_id))
        return temp_node
        # print(yaml.dump(tree, default_flow_style=False))

    def flush_class_memory(self):
        self.motion_testing_success = False
        self.child_dict = {}
        self.sorted_child_dict = {}
        self.tree = None
        self.id = ""
        self.tasks = []

# ================================================================= MAIN FUNC =============
    def recv_execute(self, tree_node):
        current_control_node_type = tree_node['data']['ros_payload']['data']
        print(tree_node['id'], current_control_node_type)
        number_of_child = len(tree_node['data']['ros_payload']['children'])
        # print("==========", current_control_node_type, tree_node['id'])
        requests.post(NODEJS_MISSION_CONTROL_URL + self.id, json={'task_id': tree_node['id'], 'task_status': "queued"})
        for idx, x in enumerate(tree_node['data']['ros_payload']['children']):
            if x['data']['ros_payload']['task'] == "control_node":
                control_node_result = self.recv_execute(x)
                # print("=======", x['id'], control_node_result, current_control_node_type == "sequence")
                if current_control_node_type == 'fallback':
                    # return True if anything inside of a fallback is true
                    if control_node_result:
                        print("reached fallback but control_node sucess")
                        requests.post(NODEJS_MISSION_CONTROL_URL + self.id,
                                      json={'task_id': tree_node['id'], 'task_status': "completed"})
                        return True
                elif current_control_node_type == 'sequence':
                    # return False if anything inside of a sequence is false
                    if not control_node_result:
                        print("reached sequence but control_node failed")
                        requests.post(NODEJS_MISSION_CONTROL_URL + self.id,
                                      json={'task_id': tree_node['id'], 'task_status': "errorred"})
                        return False
                if idx == number_of_child - 1:
                    # reaches the end
                    print("endgame", current_control_node_type)
                    if current_control_node_type == "sequence":
                        print("reaches end of sequence")
                        requests.post(NODEJS_MISSION_CONTROL_URL + self.id,
                                      json={'task_id': tree_node['id'], 'task_status': "completed"})
                        return True
                    elif current_control_node_type == "fallback":
                        requests.post(NODEJS_MISSION_CONTROL_URL + self.id,
                                      json={'task_id': tree_node['id'], 'task_status': "errorred"})
                        return False
                    elif current_control_node_type == "start":
                        if control_node_result:
                            requests.post(NODEJS_MISSION_CONTROL_URL + self.id,
                                      json={'task_id': tree_node['id'], 'task_status': "completed"})
                            return True
                        else:
                            requests.post(NODEJS_MISSION_CONTROL_URL + self.id,
                                      json={'task_id': tree_node['id'], 'task_status': "errorred"})
                            return False
            else:
                print("found not control", x['id'], x['data']['ros_payload']
                      ['task'], current_control_node_type, number_of_child)
                requests.post(NODEJS_MISSION_CONTROL_URL + self.id, json={'task_id': x['id'], 'task_status': "queued"})
                req = MotionServiceRequest()
                req.motion_task = x['data']['ros_payload']['task']
                req.payload = str(x['data']['ros_payload']['data'])
                motion_result = self.motion_client.call(req).success
                # motion_result = bool(random.getrandbits(1))
                if x['id'] == 3 :
                    motion_result = False
                if motion_result:
                    requests.post(NODEJS_MISSION_CONTROL_URL + self.id,
                                  json={'task_id': x['id'], 'task_status': "completed"})
                else:
                    requests.post(NODEJS_MISSION_CONTROL_URL + self.id,
                                  json={'task_id': x['id'], 'task_status': "errorred"})
                if current_control_node_type == 'fallback':
                    # return True if anything inside of a fallback is true
                    if motion_result:
                        print("reached fallback but motion sucess")
                        requests.post(
                            NODEJS_MISSION_CONTROL_URL + self.id,
                            json={'task_id': tree_node['id'],
                                  'task_status': "completed"})
                        return True
                elif current_control_node_type == 'sequence':
                    # return False if anything inside of a sequence is false
                    if not motion_result:
                        print("reached sequence but motion failed")
                        requests.post(
                            NODEJS_MISSION_CONTROL_URL + self.id,
                            json={'task_id': tree_node['id'],
                                  'task_status': "errorred"})
                        return False
                if idx == number_of_child - 1:
                    # reaches the end
                    if current_control_node_type == "sequence":
                        print("reaches end of sequence")
                        requests.post(
                            NODEJS_MISSION_CONTROL_URL + self.id,
                            json={'task_id': tree_node['id'],
                                  'task_status': "completed"})
                        return True
                    elif current_control_node_type == "fallback":
                        requests.post(
                            NODEJS_MISSION_CONTROL_URL + self.id,
                            json={'task_id': tree_node['id'],
                                  'task_status': "errorred"})
                        return False
    # requests.post(NODEJS_MISSION_CONTROL_URL + id,
    #                             json={'task_id': task['id'], 'task_status': "completed"})

    def single_executor(self, task):
        success = False
        print(task['data']['ros_payload'])
        if task['data']['ros_payload']['service'] == "motion_service":
            req = MotionServiceRequest()
            req.motion_task = task['data']['ros_payload']['task']
            req.payload = str(task['data']['ros_payload']['data'])
            print(req)
            motion_result = self.motion_client.call(req)
            self.motion_testing_success = motion_result.success

    def execute_cb(self, req):
        self.flush_class_memory()
        temp_req_json = json.loads(req.json_data)
        self.id = temp_req_json['_id']
        self.tasks = temp_req_json['task']

        if self.id == "ui_issued_testing":
            if len(self.tasks) != 1:
                rospy.logerr("Bad treenode request, ui testing treenode should contain only one node")
                return TreeExecutorServiceResponse(success=False)

            motion_thread = Thread(target=self.single_executor, args=(
                self.tasks,))
            motion_thread.start()
            motion_thread.join()
            return TreeExecutorServiceResponse(success=self.motion_testing_success)

        else:
            for task in self.tasks:
                if task['parent'] in self.child_dict.keys():
                    self.child_dict[task['parent']].append(task['id'])
                else:
                    self.child_dict[task['parent']] = [task['id']]
            for i in sorted(self.child_dict.keys()):
                self.sorted_child_dict[i] = self.child_dict[i]

            print("sorted_child_dict", self.sorted_child_dict)

            if len(self.sorted_child_dict[0]) == 1:
                self.tree = self.tasks[self.search_dict_by_id(self.sorted_child_dict[0][0])]
                self.tree = self.build_tree(self.sorted_child_dict[0][0])
                # print(yaml.dump(self.tree, default_flow_style=False))

                motion_thread = Thread(target=self.recv_execute, args=(
                    self.tree,))
                motion_thread.start()
                motion_thread.join()
                return TreeExecutorServiceResponse(success=True)

        # if mission['_id'] == "motion_testing":
        #     MOTION_TESTING_SUCCESS = False
        #     motion_thread.join()
        #     return TreeExecutorServiceResponse(success=MOTION_TESTING_SUCCESS)
        # else:
        #     return TreeExecutorServiceResponse(success=True)


'''
def load_tree(self):
    rospack = rospkg.RosPack()
    test_json_file = open(rospack.get_path(
        'tree_executor') + '/config/sample_output.json')
    data = json.load(test_json_file)
    return (data['_id'], data['task'])




def recv_execute_bugged(tree_node):
    print("running {}".format(tree_node['text']))
    if tree_node['data']['ros_payload']['data'] == "sequence":
        print("encountered a sequence node")
        for x in tree_node['data']['ros_payload']['children']:
            if not recv_execute(x):
                return False
    elif tree_node['data']['ros_payload']['data'] == "fallback":
        print("encountered a fallback node")
        for x in tree_node['data']['ros_payload']['children']:
            if recv_execute(x):
                return True
    else:
        for x in tree_node['data']['ros_payload']['children']:
            recv_execute(x)
    if not tree_node['data']['ros_payload']['data'] == "start":
        req = MotionServiceRequest()
        req.motion_task = tree_node['data']['ros_payload']['task']
        req.payload = str(tree_node['data']['ros_payload']['data'])
        print(req.payload)
        motion_result = motion_client.call(req)
        return motion_result.success

def debug():
    global sorted_child_dict, tasks, tree
    id, tasks = load_tree()
    child_dict = {}

    for task in tasks:
        if task['parent'] in child_dict.keys():
            child_dict[task['parent']].append(task['id'])
        else:
            child_dict[task['parent']] = [task['id']]
    for i in sorted(child_dict.keys()):
        sorted_child_dict[i] = child_dict[i]
    print(sorted_child_dict)
    if len(sorted_child_dict[0]) == 1:
        tree = tasks[search_dict_by_id(sorted_child_dict[0][0])]
        tree = build_tree(sorted_child_dict[0][0])
        print(yaml.dump(tree, default_flow_style=False))

        recv_execute(tree)


'''
if __name__ == "__main__":
    rospy.init_node("tree")
    tree_executor = TreeExeClass()
    rospy.spin()
    # debug()

# {0: [1], 1: [2], 2: [8, 5, 6, 7], 8: [3, 4]}
