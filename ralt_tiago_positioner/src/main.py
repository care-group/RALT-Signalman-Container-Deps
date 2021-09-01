#! /usr/bin/env python3
import rospy
import threading
from time import time, strftime, sleep
import subprocess

from flask import Flask, request, jsonify
from flask_cors import CORS

from geometry_msgs.msg import PoseStamped

class Main():
    def __init__(self):
        self.id = 'main'

        self.pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)   

        self.msg = False

        print('Ready.')

    def loop(self):
        while True:
            if self.msg:
                self.send_to_tiago_pub(self.pos)
                self.msg = False
            else:
                rospy.rostime.wallsleep(0.5)

    def send_to_tiago(self, pos):
        self.pos = pos
            
    def send_to_tiago_pub(self, pos):
        goal_msg = PoseStamped()

        goal_msg.header.frame_id = "map"
        goal_msg.header.stamp = rospy.Time.now()

        pose = self.get_pose(pos)
        print('Sending Tiago to...', pose[0], pose[1])

        goal_msg.pose.position.x = pose[0]
        goal_msg.pose.position.y = pose[1]
        goal_msg.pose.position.z = pose[2]

        goal_msg.pose.orientation.w = 1.0

        self.pub.publish(goal_msg)

    def get_pose(self, pos):
        pose = []
        if pos == 1:
            pose.append(-1.0)
            pose.append(-1.0)
            pose.append(0.0)
        elif pos == 2:
            pose.append(0.0)
            pose.append(-1.0)
            pose.append(0.0)
        elif pos == 3:
            pose.append(1.0)
            pose.append(0.0)
            pose.append(0.0)
        elif pos == 4:
            pose.append(1.5)
            pose.append(-1.0)
            pose.append(0.0)
        else:
            pose.append(-1.0)
            pose.append(-1.0)
            pose.append(0.0)

        return pose

if __name__ == '__main__':
    threading.Thread(target=lambda: rospy.init_node('ralt_tiago_positioner', disable_signals=True)).start()

    m = Main()

    app = Flask(__name__)
    CORS(app)
    
    print('Delayed start. Sleeping for 10 seconds...')
    sleep(1)

    threading.Thread(target=lambda: m.loop()).start()
    
    @app.route('/control', methods = ['POST'])
    def control_handler():
        data = request.get_data()
        data = int(data)
        print(data)

        resp = "OK"

        if data == 1:
            m.send_to_tiago(1)
            m.msg = True
        elif data == 2:
            m.send_to_tiago(2)
            m.msg = True
        elif data == 3:
            m.send_to_tiago(3)
            m.msg = True
        elif data == 4:
            m.send_to_tiago(4)
            m.msg = True
        else:
            resp = "Invalid request. You have requested an invalid position."

        return resp

    @app.route('/status', methods = ['GET'])
    def status_handler():
        status = "OK"

        # @TODO: Allow for checking status of Tiago (e.g. planning, doing, stopped, done)

        return jsonify(status)

    app.run(host='0.0.0.0', port = 5005)