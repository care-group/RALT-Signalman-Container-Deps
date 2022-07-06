#! /usr/bin/env python

import rospy
import threading
import socket
import time

from flask import Flask, request, jsonify
from flask_cors import CORS

from csv_tools import CSVTools
from check_create_folder_tool import FolderCheckCreate

from std_msgs.msg import String

HOST = '0.0.0.0'
PORT = 5001

class AppleWatchPublisher():
    def __init__(self):
        self.run = False

        self.activity = 'none'

        self.participant = 999

        print('Ready.')

    def loop(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        s.bind((HOST, PORT))
        s.listen(1)
        
        while True:
            if self.run:
                fcc = FolderCheckCreate()
                fcc.run(self.participant)
                
                conn, client = s.accept()
            
                csvt = CSVTools()
                csvt.create_event_file(self.activity, self.participant)

                try:
                    print('Client connected. Waiting for data...')
                    while True:
                        data = conn.recv(1024)
                        if data:
                            print(data)
                            message = str(data)
                            
                            timestamp_s = time.time()
                            timestamp_formatted = time.ctime(timestamp_s)

                            event = [data, timestamp_s, timestamp_formatted]
                            events = [event]
                            csvt.write_events(events)
                            
                            if not self.run:
                                break
                        else:
                            print('Client disconnect. Listening for new connection...')
                            break

                        if not rospy.is_shutdown():
                                pub.publish(message)
                        else:
                            break
                finally:
                    conn.close()

    def set_state(self, cmd):
        if cmd:
            self.run = True
        else:
            self.run = False

    def set_activity(self, activity):
        self.activity = activity

    def set_participant(self, participant):
        self.participant = participant
        fcc = FolderCheckCreate()
        fcc.run(self.participant)

if __name__ == '__main__':
    threading.Thread(target=lambda: rospy.init_node('aw_pub', disable_signals=True)).start()
    pub = rospy.Publisher('ralt_apple_watch_publisher', String, queue_size=10)

    awp = AppleWatchPublisher()

    app = Flask(__name__)
    CORS(app)

    threading.Thread(target=lambda: awp.loop()).start()

    @app.route('/control', methods = ['POST'])
    def control_handler():
        data = request.get_json()
        
        command = data['command']
        activity = data['activity']

        print(command, activity)

        awp.set_activity(activity)

        resp = "OK"

        if command == "True":
            awp.set_state(True)
        elif command == "False":
            awp.set_state(False)
        else:
            resp = "Invalid state. Send command to either 'True' or 'False'."

        return resp

    @app.route('/participant', methods = ['POST'])
    def participant_handler():
        data = request.get_json()

        participant = data['participant']

        participant = int(participant)

        awp.set_participant(participant)

        resp = "OK"

        return resp

    @app.route('/status', methods = ['GET'])
    def status_handler():
        status = "OK"

        return jsonify(status)

    app.run(host='0.0.0.0', port = 5008)
