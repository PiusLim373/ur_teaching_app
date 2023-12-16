#!/usr/bin/env python
import rospy
from flask import Flask, request, jsonify
from threading import Thread

class FlaskInterfaces():
    def __init__(self, flask_app, params=None, base_path=None,server_ip=None):
        pass

if __name__ == "__main__":
    rospy.init_node("rest_interface")
    app = Flask(rospy.get_name())
    # main()