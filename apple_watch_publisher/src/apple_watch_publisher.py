#! /usr/bin/env python

import rospy
import threading
import socket
import time

from flask import Flask, request, jsonify
from flask_cors import CORS

from csv_tools import CSVTools

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
        while True:
            if self.run:
                csvt = CSVTools()
                csvt.create_event_file(self.activity, self.participant)

                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

                s.bind((HOST, PORT))
                s.listen(1)

                conn, client = s.accept()

                try:
                    print('Client connected. Waiting for data...')
                    while True:
                        data = conn.recv(1024)
                        if data:
                            print(data)
                            message = str(data)
                            
                            timestamp_ms = time.time()
                            timestamp_formatted = time.ctime(timestamp_ms)

                            event = [data, timestamp_ms, timestamp_formatted]
                            events = [event]
                            csvt.write_events(events)
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

if __name__ == '__main__':
    threading.Thread(target=lambda: rospy.init_node('aw_pub', disable_signals=True)).start()
    pub = rospy.Publisher('apple_watch_publisher', String, queue_size=10)

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