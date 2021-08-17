#! /usr/bin/env python
import rospy
import threading
from time import time, strftime, sleep
import subprocess

from flask import Flask, request, jsonify
from flask_cors import CORS
from std_msgs import msg

from std_msgs.msg import String, Int32
from sensor_msgs.msg import Image, CompressedImage

class Main():
    def __init__(self):
        self.id = 'main'

        self.date_time = strftime("%Y%m%d-%H%M%S")

        self.topics = []

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
                self.topics.append(splits[0])
            
            print('Will check for messages on these topics...')
            print(self.topics)
            
    def loop(self):
        while(True):
            if self.run:
                date_time = strftime("%Y%m%d-%H%M%S")
                self.bag_name = '/home/sandbox/shared/output/data_' + date_time + '.bag'

                while(self.run):
                    cmd = ['rosbag', 'record', '-O', self.bag_name]
                    for topic in self.topics:
                        cmd.append(topic)
                    
                    self.p = subprocess.Popen(cmd)

                    while not rospy.core.is_shutdown() and self.run:
                        rospy.rostime.wallsleep(0.5)

                self.p.terminate()            

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
    CORS(app)
    
    print('Delayed start. Sleeping for 10 seconds...')
    sleep(10)

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
