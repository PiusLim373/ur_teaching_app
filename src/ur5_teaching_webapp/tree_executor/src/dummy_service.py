#!/usr/bin/env python
import rospy
import json
from tree_executor.srv import DummyService, DummyServiceResponse
from ur5_pap.srv import *

def service_cb(req):
    print(req)
    return_data =  MotionServiceResponse()
    return_data.success = True
    print(return_data)
    return return_data

def main():
    rospy.init_node("tree")
    rospy.Service('dummy_service', MotionService, service_cb)
    print("Ready to add two ints.")
    rospy.spin()


if __name__ == "__main__":
    main()