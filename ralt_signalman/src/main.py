#! /usr/bin/env python
import rospy
import rosbag
import threading
from time import time, strftime, sleep

from flask import Flask, request, jsonify
from std_msgs import msg

from std_msgs.msg import String, Int32
from sensor_msgs.msg import Image

class Main():
    def __init__(self):
        self.id = 'main'

        self.date_time = strftime("%Y%m%d-%H%M%S")

        self.topics = []
        self.msg_types = []

        self.run = False
        self.bag_name = 'No current or previous bag has been saved.'

        self.load_topics()

        print('Ready.')

    def load_topics(self):
        with open('/home/sandbox/shared/input/topics.txt') as f:
            lines = f.readlines()

            lines_clean = []
            for line in lines:
                lines_clean.append(line.rstrip('\n'))

            for line in lines_clean:
                splits = line.split(', ')
                self.topics.append(splits[0])
                self.msg_types.append(splits[1])
            
            print('Will check for messages on these topics...')
            print(self.topics, self.msg_types)

    def callback(self, data, topic):
        if self.run:
            self.bag.write(topic, data)
            print("I heard from topic:", topic, "who said:", data)

    def loop(self):
        while(True):
            if self.run:
                date_time = strftime("%Y%m%d-%H%M%S")
                self.bag_name = '/home/sandbox/shared/output/data_' + date_time + '.bag'
                self.bag = rosbag.Bag(self.bag_name, 'w')

                while(self.run):
                    for topic, msg_type in zip(self.topics, self.msg_types):
                        if msg_type == "String":
                            rospy.Subscriber(topic, String, self.callback, callback_args=topic)
                        elif msg_type == "Image":
                            rospy.Subscriber(topic, Image, self.callback, callback_args=topic)
                        elif msg_type == "Int32":
                            rospy.Subscriber(topic, Int32, self.callback, callback_args=topic)
                    while not rospy.core.is_shutdown() and self.run:
                        rospy.rostime.wallsleep(0.5)

                self.bag.close()            

            print('Not recording.')
            sleep(1)

    def set_state(self, cmd):
        if cmd:
            self.run = True
        else:
            self.run = False

if __name__ == '__main__':
    threading.Thread(target=lambda: rospy.init_node('ralt_signalman', disable_signals=True)).start()

    m = Main()

    app = Flask(__name__)

    threading.Thread(target=lambda: m.loop()).start()

    @app.route('/control', methods = ['POST'])
    def control_handler():
        data = request.get_data()
        print(data)

        resp = "OK"

        if data == "True":
            m.set_state(True)
        elif data == "False":
            m.set_state(False)
        else:
            resp = "Invalid state. Send either 'True' or 'False'."

        return resp

    @app.route('/status', methods = ['GET'])
    def status_handler():
        status = {}

        status["topics"] = m.topics
        status["running"] = m.run
        status["bagfile"] = m.bag_name

        return jsonify(status)

    app.run(host='0.0.0.0', port = 5002)