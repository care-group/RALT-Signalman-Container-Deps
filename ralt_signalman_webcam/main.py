#! /usr/bin/env python
import threading
from time import time, strftime, sleep
import subprocess

from flask import Flask, request, jsonify
from flask_cors import CORS

class Main():
    def __init__(self):
        self.id = 'main'

        self.filenames = []
        self.filenames.append('No current or previous camera recordings have been saved.')
        self.filenames.append('No current or previous camera recordings have been saved.')
        self.filenames.append('No current or previous camera recordings have been saved.')

        self.run = False

        print('Ready.')

    def loop(self):
        while(True):
            if self.run:
                date_time = strftime("%Y%m%d-%H%M%S")

                self.filenames = []

                fn_1 = '/home/sandbox/shared/output/camera_1_' + date_time + '.mp4'
                fn_2 = '/home/sandbox/shared/output/camera_2_' + date_time + '.mp4'
                fn_3 = '/home/sandbox/shared/output/camera_3_' + date_time + '.mp4'

                self.filenames.append(fn_1)
                self.filenames.append(fn_2)
                self.filenames.append(fn_3)

                while(self.run):
                    cmd_1 = ['ffmpeg', '-f', 'v4l2', '-framerate', '25', '-video_size', '1920x1080', '-input_format', 'mjpeg', '-i', '/dev/video0', self.filenames[0]]
                    cmd_2 = ['ffmpeg', '-f', 'v4l2', '-framerate', '25', '-video_size', '1920x1080', '-input_format', 'mjpeg', '-i', '/dev/video2', self.filenames[1]]
                    cmd_3 = ['ffmpeg', '-f', 'v4l2', '-framerate', '25', '-video_size', '1920x1080', '-input_format', 'mjpeg', '-i', '/dev/video4', self.filenames[2]]

                    self.p_1 = subprocess.Popen(cmd_1)
                    self.p_2 = subprocess.Popen(cmd_2)
                    self.p_3 = subprocess.Popen(cmd_3)

                    while not rospy.core.is_shutdown() and self.run:
                        rospy.rostime.wallsleep(0.5)

                self.p_1.terminate()
                self.p_2.terminate()
                self.p_3.terminate()            

            sleep(1)

    def set_state(state):
        self.run = state

if __name__ == '__main__':
    m = Main()

    app = Flask(__name__)
    CORS(app)
    
    print('Delayed start. Sleeping for 10 seconds...')
    sleep(1)

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
            resp = "Invalid control message. Either True or False to start/stop recording."

        return resp

    @app.route('/status', methods = ['GET'])
    def status_handler():
        status = {}

        status["camera_1"] = self.filenames[0]
        status["camera_2"] = self.filenames[1]
        status["camera_3"] = self.filenames[2]

        return jsonify(status)

    app.run(host='0.0.0.0', port = 5006)