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

        self.activity = 'none'

        print('Ready.')

    def loop(self):
        while(True):
            if self.run:
                date_time = strftime("%Y%m%d-%H%M%S")

                self.filenames = []

                if self.activity == 'none':
                    fn_1 = '/home/sandbox/output/camera_1_' + date_time + '.mkv'
                    fn_2 = '/home/sandbox/output/camera_2_' + date_time + '.mkv'
                    fn_3 = '/home/sandbox/output/camera_3_' + date_time + '.mkv'
                else:
                    fn_1 = '/home/sandbox/output/camera_1_' + date_time + '_' + self.activity + '.mkv'
                    fn_2 = '/home/sandbox/output/camera_2_' + date_time + '_' + self.activity + '.mkv'
                    fn_3 = '/home/sandbox/output/camera_3_' + date_time + '_' + self.activity + '.mkv'

                self.filenames.append(fn_1)
                self.filenames.append(fn_2)
                self.filenames.append(fn_3)

                cmd_1 = ['ffmpeg', '-f', 'v4l2', '-framerate', '25', '-video_size', '1920x1080', '-input_format', 'mjpeg', '-i', '/dev/video0', self.filenames[0]]
                cmd_2 = ['ffmpeg', '-f', 'v4l2', '-framerate', '25', '-video_size', '1920x1080', '-input_format', 'mjpeg', '-i', '/dev/video2', self.filenames[1]]
                cmd_3 = ['ffmpeg', '-f', 'v4l2', '-framerate', '25', '-video_size', '1920x1080', '-input_format', 'mjpeg', '-i', '/dev/video4', self.filenames[2]]

                self.p_1 = subprocess.Popen(cmd_1)
                self.p_2 = subprocess.Popen(cmd_2)
                self.p_3 = subprocess.Popen(cmd_3)

                while self.run:
                    sleep(1)
                    
                self.p_1.terminate()
                self.p_2.terminate()
                self.p_3.terminate()            

            sleep(1)

    def set_state(self, state):
        self.run = state

    def set_activity(self, activity):
        self.activity = activity

if __name__ == '__main__':
    m = Main()

    app = Flask(__name__)
    CORS(app)
    
    print('Delayed start. Sleeping for 10 seconds...')
    sleep(1)

    threading.Thread(target=lambda: m.loop()).start()

    @app.route('/control', methods = ['POST'])
    def control_handler():
        data = request.get_json()
        
        command = data['command']
        activity = data['activity']

        print(command, activity)

        resp = "OK"

        m.set_activity(activity)

        if command == "True":
            m.set_state(True)
        elif command == "False":
            m.set_state(False)
        else:
            resp = "Invalid state. Send command to either 'True' or 'False'."

        return resp

    @app.route('/status', methods = ['GET'])
    def status_handler():
        status = {}

        status["camera_1"] = m.filenames[0]
        status["camera_2"] = m.filenames[1]
        status["camera_3"] = m.filenames[2]

        return jsonify(status)

    app.run(host='0.0.0.0', port = 5006)
