#! /usr/bin/env python
import rospy
import threading
from time import sleep, strftime, time

from log import Log
from load_yaml import LoadYAML
from openhab_helper import OpenHABHelper
from csv_tools import CSVTools
from detect_events import DetectEvents

from flask import Flask, request, jsonify
from flask_cors import CORS

from check_create_folder_tool import FolderCheckCreate

from std_msgs.msg import String
from ralt_signalman_messages.msg import har_simple_evidence

SENSORS_CONFIG = "sensors.yaml"
PERIODICITY = 1.0
OPENHAB_URL = "http://0.0.0.0:8080/rest/items"

class Main():
    def __init__(self, pub_full, pub_simple):
        self.id = 'main'
        
        self.logger = Log(self.id)
        self.logger.startup_msg()
        
        rospy.init_node('ralt_semantic_event_logger', disable_signals=True)

        self.run = False

        self.pub_full = pub_full
        self.pub_simple = pub_simple

        # * * * CONFIGURATION AREA * * * 

        # set to True for real-time mode (1 second/sample)
        self.real_time = True
        
        self.debug = False

        # * * * * * * * * * * * * * * *

        # Load YAML
        self.load_yaml = LoadYAML()
        self.sensors, self.sensor_labels = self.load_yaml.load_file(SENSORS_CONFIG)

        # Set up OpenHAB helper
        self.openhab_helper = OpenHABHelper(OPENHAB_URL, self.sensor_labels)

        # Set up event detector
        self.detect_events = DetectEvents(self.sensors, self.sensor_labels)

        self.step = 0

        self.activity = 'none'

        self.participant = 999

        self.logger.log_great('Ready.')

    def loop(self):
        while(True):
            if self.run:
                print('Publish and log mode...')
                
                fcc = FolderCheckCreate()
                fcc.run(self.participant)
                
                self.csv_tools = CSVTools()
                self.csv_tools.create_event_file(self.activity, self.participant)

                while(self.run):
                    if self.real_time:
                        start_time = time()

                    self.current_state = False
                    while not self.current_state:
                        self.current_state = self.openhab_helper.update()
                    
                    if self.step == 0:
                        self.detect_events.init_semantic_state(self.current_state)

                    if self.step > 0:
                        events = self.detect_events.step(self.current_state, self.previous_state, self.step)
                        if len(events) > 0:
                            self.csv_tools.write_events(events)
                            if not rospy.is_shutdown():
                                for event in events:
                                    msg = str(event)
                                    self.pub_full.publish(msg)
                                    
                                    evidence = event[4]
                                    etype = 'event'

                                    msg = har_simple_evidence()
                                    msg.evidence = evidence
                                    msg.etype = etype
                                    msg.cmd = 'add'

                                    self.pub_simple.publish(msg)

                    if self.real_time:
                        end_time = time()
                        time_taken = end_time - start_time
                        delay_time = PERIODICITY - time_taken

                        if delay_time >= 0.0:
                            if self.debug:
                                msg = 'Loop time was: ' + str(time_taken) + ', sleeping for: ' + str(delay_time) + ' seconds.'
                                print(msg)
                            sleep(delay_time)
                        else:
                            print('Loop took longer than the specified period. System is not able to perform in real-time at this periodicity.')

                    self.previous_state = self.current_state
                    self.step = self.step + 1
            else:
                print('Publish only mode...')
                while(not self.run):
                    self.current_state = False
                    while not self.current_state:
                        self.current_state = self.openhab_helper.update()

                    if self.step == 0:
                        self.detect_events.init_semantic_state(self.current_state)

                    if self.step > 0:
                        events = self.detect_events.step(self.current_state, self.previous_state, self.step)
                        if len(events) > 0:
                            if not rospy.is_shutdown():
                                for event in events:
                                    msg = str(event)
                                    self.pub_full.publish(msg)

                                    evidence = event[4]
                                    etype = 'event'

                                    msg = har_simple_evidence()
                                    msg.evidence = evidence
                                    msg.etype = etype
                                    msg.cmd = 'add'
                                    
                                    print('Sending to /ralt_semantic_event_publisher/simple', evidence, etype)

                                    self.pub_simple.publish(msg)

                    self.previous_state = self.current_state
                    self.step = self.step + 1

    def set_state(self, cmd):
        if cmd:
            self.run = True
            self.step = 0
        else:
            self.run = False
            self.step = 0

    def set_activity(self, activity):
        self.activity = activity

    def set_participant(self, participant):
        self.participant = participant
        fcc = FolderCheckCreate()
        fcc.run(self.participant)

if __name__ == '__main__':
    pub_full = rospy.Publisher('ralt_semantic_event_publisher/full', String, queue_size=10)
    pub_simple = rospy.Publisher('ralt_semantic_event_publisher/simple', har_simple_evidence, queue_size=10)
    
    m = Main(pub_full, pub_simple)

    app = Flask(__name__)
    CORS(app)

    threading.Thread(target=lambda: m.loop()).start()

    @app.route('/control', methods = ['POST'])
    def control_handler():
        data = request.get_json()
        
        command = data['command']
        activity = data['activity']

        resp = "OK"

        m.set_activity(activity)

        if command == "True":
            m.set_state(True)
        elif command == "False":
            m.set_state(False)
        else:
            resp = "Invalid state. Send command to either 'True' or 'False'."

        return resp

    @app.route('/participant', methods = ['POST'])
    def participant_handler():
        data = request.get_json()

        participant = data['participant']

        participant = int(participant)

        m.set_participant(participant)

        resp = "OK"

        return resp

    @app.route('/status', methods = ['GET'])
    def status_handler():
        status = "OK"

        return jsonify(status)

    app.run(host='0.0.0.0', port = 5009)
